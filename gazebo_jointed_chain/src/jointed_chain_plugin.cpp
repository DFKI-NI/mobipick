// A jointed chain between two links of a Gazebo model (e.g. a cable carrier between two clamps of a robot arm).
//
// The chain is its own model, built at load from parameters: N rigid capsule links of equal length, joined by
// universal joints (two bend axes with limits, no twist; ODE ball joints have no limits) with damping, hanging in a
// circular arc below the chord between the two clamp points; its first and last link are held at the clamps by
// joints to the parent and child link. It collides only with the host collisions listed in <collide_with> (a token
// anywhere in the collision name): they get the chain's bit alone (they touch nothing but the chain), every other
// collision of the host model and, with <exclusive> (default), of every other model loses that bit.
//
// Published at <rate> Hz on <ns>/<name>/state (std_msgs/Float64MultiArray, layout in the README): the clamp forces,
// the chain's strain (length through the joint points / free length - 1) and the link end points in <reference_link>.
// <ns>/<name>/reset (std_srvs/Empty) hangs the chain afresh below its chord. Optional: above <stop_strain> or
// <stop_force> for <stop_ticks> published ticks the plugin calls the std_srvs/SetBool service <stop_service> with true
// (once; again after the values were back below).
//
// Light links next to a heavy arm make ODE's quickstep unstable: <inertia_scale> multiplies the links' inertia (the
// mass stays), 4 was stable at quickstep 200 iterations, 3 ms (#263 step 1).
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace gazebo
{
using ignition::math::Matrix3d;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;

namespace
{
template <typename T>
T param(const sdf::ElementPtr &sdf, const std::string &key, const T &fallback)
{
  return sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
}

// the arc angle t of a circular arc of the given length over the given chord: sin(t/2) / (t/2) = chord / length
double arcAngle(double chord, double length)
{
  double lo = 1e-6, hi = 2 * M_PI - 1e-6;
  for (int i = 0; i < 100; ++i)
  {
    const double mid = 0.5 * (lo + hi);
    if (std::sin(mid / 2) / (mid / 2) > chord / length)
      lo = mid;
    else
      hi = mid;
  }
  return 0.5 * (lo + hi);
}
}  // namespace

class JointedChainPlugin : public ModelPlugin
{
public:
  ~JointedChainPlugin() override
  {
    if (ros_)
      ros_->shutdown();
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    world_ = model->GetWorld();
    // the model whose links hold the chain: this one, or <host_model> (spawned before or after this plugin's model)
    host_name_ = param<std::string>(sdf, "host_model", "");
    own_ = model;
    name_ = param<std::string>(sdf, "chain_name", (host_name_.empty() ? model->GetName() : host_name_) + "_chain");
    parent_name_ = param<std::string>(sdf, "parent_link", "");
    child_name_ = param<std::string>(sdf, "child_link", "");   // empty: the far end hangs free
    reference_name_ = param<std::string>(sdf, "reference_link", "");
    parent_offset_ = param<Pose3d>(sdf, "parent_offset", Pose3d::Zero);
    child_offset_ = param<Pose3d>(sdf, "child_offset", Pose3d::Zero);
    n_ = std::max(2, param<int>(sdf, "links", 15));
    free_length_ = param<double>(sdf, "length", 0.52);
    link_length_ = free_length_ / n_;
    radius_ = param<double>(sdf, "radius", 0.0175);
    mass_ = param<double>(sdf, "mass_per_link", 0.0122);
    inertia_scale_ = param<double>(sdf, "inertia_scale", 8.0);
    // the bend limit per axis: link length / minimum bend radius (bend_radius wins over bend_limit when given)
    bend_limit_ = sdf->HasElement("bend_radius") ? link_length_ / sdf->Get<double>("bend_radius")
                                                 : param<double>(sdf, "bend_limit", 0.35);
    exclusive_ = param<bool>(sdf, "exclusive", true);
    damping_ = param<double>(sdf, "damping", 0.002);
    // bending stiffness EI [N m^2]: a spring per bend axis to the straight shape, k = EI / link length (implicit in
    // ODE, stable with the light links); 0 = free hinges (a rope)
    bending_stiffness_ = param<double>(sdf, "bending_stiffness", 0.0);
    friction_ = param<double>(sdf, "friction", 0.3);
    bitmask_ = static_cast<unsigned int>(param<int>(sdf, "collide_bitmask", 0x8000));
    rate_ = param<double>(sdf, "rate", 10.0);
    stop_strain_ = param<double>(sdf, "stop_strain", 0.0);
    stop_force_ = param<double>(sdf, "stop_force", 0.0);
    stop_ticks_ = param<int>(sdf, "stop_ticks", 3);
    stop_service_ = param<std::string>(sdf, "stop_service", "");
    reference_offset_ = param<Pose3d>(sdf, "reference_offset", Pose3d::Zero);
    // a shape bent more than the joints allow (an initial shape from a tighter carrier model) would put joints past
    // their limits, which ODE never resolves (the chain stays stretched): such joints start with their limit widened
    // to their bend and the limits narrow to bend_limit over settle_time [s] of sim time (no stop call meanwhile), so
    // the chain moves into a shape it can hold under physics, the shapes still in place
    settle_time_ = param<double>(sdf, "settle_time", 2.0);
    // <initial_points>x y z x y z ...</initial_points>: the chain's shape at spawn, in the parent link frame (e.g. a
    // settled model of the same carrier, so it starts on the right side of every shape); resampled to the links. Used
    // while its ends are within 2 cm of the clamps, else the arc below the chord
    if (sdf->HasElement("initial_points"))
    {
      std::istringstream in(sdf->Get<std::string>("initial_points"));
      std::vector<double> v;
      for (double x; in >> x;)
        v.push_back(x);
      for (size_t i = 0; i + 2 < v.size(); i += 3)
        initial_points_.emplace_back(v[i], v[i + 1], v[i + 2]);
      if (initial_points_.size() < 2)
        initial_points_.clear();
    }
    if (sdf->HasElement("collide_with"))
    {
      std::istringstream names(sdf->Get<std::string>("collide_with"));
      for (std::string token; names >> token;)
        keep_.push_back(token);
    }
    // <shape><name/><link/><type>box|cylinder|sphere</type><pose/><size/></shape>: collisions added to host links at
    // runtime that touch only the chain (e.g. the shapes a cable carrier runs over), so the host's own description
    // stays unchanged
    for (sdf::ElementPtr e = sdf->HasElement("shape") ? sdf->GetElement("shape") : nullptr; e;
         e = e->GetNextElement("shape"))
      shapes_.push_back({param<std::string>(e, "name", "shape" + std::to_string(shapes_.size())),
                         param<std::string>(e, "link", ""), param<std::string>(e, "type", "box"),
                         param<Pose3d>(e, "pose", Pose3d::Zero), param<std::string>(e, "size", "0.01 0.01 0.01")});

    if (ros::isInitialized())
    {
      ros_.reset(new ros::NodeHandle(param<std::string>(sdf, "robot_namespace", "")));
      state_pub_ = ros_->advertise<std_msgs::Float64MultiArray>(name_ + "/state", 1);
      reset_srv_ = ros_->advertiseService(name_ + "/reset", &JointedChainPlugin::onReset, this);
      if (!stop_service_.empty())
        stop_client_ = ros_->serviceClient<std_srvs::SetBool>(stop_service_);
    }
    else
      gzwarn << "gazebo_jointed_chain: ROS is not initialized (load gazebo_ros), no state topic\n";
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&JointedChainPlugin::onUpdate, this));
    gzmsg << "gazebo_jointed_chain: " << name_ << ", " << n_ << " links of " << link_length_ << " m between "
          << parent_name_ << " and " << (child_name_.empty() ? std::string("a free end") : child_name_) << " of "
          << (host_name_.empty() ? own_->GetName() : host_name_) << ", " << shapes_.size() << " shapes\n";
  }

private:
  // clamp points in the world
  Vector3d clampA() const { return (parent_offset_ + parent_->WorldPose()).Pos(); }
  Vector3d clampB() const
  {
    if (child_)
      return (child_offset_ + child_->WorldPose()).Pos();
    // free end: straight out along the parent clamp's x axis
    const Pose3d a = parent_offset_ + parent_->WorldPose();
    return a.Pos() + a.Rot().RotateVector(Vector3d(free_length_, 0, 0));
  }

  // the links' world poses on an arc of the free length hanging below the chord (towards gravity)
  // the initial shape in the world, resampled to n_ links of equal length; empty when its ends are more than 2 cm
  // from the clamps (the arm has moved since it was computed)
  std::vector<Vector3d> initialPoints() const
  {
    if (initial_points_.empty())
      return {};
    const Pose3d parent = parent_->WorldPose();
    std::vector<Vector3d> world;
    for (const auto &p : initial_points_)
      world.push_back(parent.Pos() + parent.Rot().RotateVector(p));
    if (world.front().Distance(clampA()) > 0.02 || world.back().Distance(clampB()) > 0.02)
      return {};
    return resample(world, n_);
  }

  std::string initialShapeText() const
  {
    const std::vector<Vector3d> p = initialPoints();
    double length = 0.0;
    for (size_t i = 1; i < p.size(); ++i)
      length += p[i].Distance(p[i - 1]);
    std::ostringstream out;
    out << "length " << length << " m for " << free_length_ << " m, largest bend " << maxBend(p) * 180 / M_PI
        << " deg, limit " << bend_limit_ * 180 / M_PI;
    return out.str();
  }

  // n + 1 points at equal distances along a polyline
  static std::vector<Vector3d> resample(const std::vector<Vector3d> &line, int n)
  {
    std::vector<double> along{0.0};
    for (size_t i = 1; i < line.size(); ++i)
      along.push_back(along.back() + line[i].Distance(line[i - 1]));
    std::vector<Vector3d> points;
    size_t k = 1;
    for (int i = 0; i <= n; ++i)
    {
      const double s = along.back() * i / n;
      while (k + 1 < line.size() && along[k] < s)
        ++k;
      const double seg = std::max(1e-9, along[k] - along[k - 1]);
      points.push_back(line[k - 1] + (line[k] - line[k - 1]) * std::min(1.0, std::max(0.0, (s - along[k - 1]) / seg)));
    }
    return points;
  }

  // the largest angle between neighbouring segments [rad]
  static double maxBend(const std::vector<Vector3d> &points)
  {
    double bend = 0.0;
    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
      const Vector3d a = (points[i] - points[i - 1]).Normalized(), b = (points[i + 1] - points[i]).Normalized();
      bend = std::max(bend, std::acos(std::min(1.0, std::max(-1.0, a.Dot(b)))));
    }
    return bend;
  }

  // link poses along points: x along each link, the other axes carried from link to link without twist
  static std::vector<Pose3d> posesAlong(const std::vector<Vector3d> &points, Vector3d y)
  {
    std::vector<Pose3d> poses;
    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
      const Vector3d x = (points[i + 1] - points[i]).Normalized();
      y = y - x * y.Dot(x);
      if (y.Length() < 1e-6)
        y = x.Perpendicular();
      y.Normalize();
      const Vector3d z = x.Cross(y);
      Matrix3d rot(x.X(), y.X(), z.X(), x.Y(), y.Y(), z.Y(), x.Z(), y.Z(), z.Z());
      poses.emplace_back((points[i] + points[i + 1]) * 0.5, Quaterniond(rot));
    }
    return poses;
  }

  std::vector<Pose3d> arcPoses() const
  {
    const std::vector<Vector3d> initial = initialPoints();
    if (!initial.empty())
    {
      const Vector3d x0 = (initial[1] - initial[0]).Normalized();
      Vector3d y0 = Vector3d(0, 0, -1).Cross(x0);
      return posesAlong(initial, y0.Length() > 1e-6 ? y0 : x0.Perpendicular());
    }
    const Vector3d a = clampA(), b = clampB();
    const double chord = std::max(1e-4, a.Distance(b));
    Vector3d u = (b - a) / chord;
    Vector3d down = world_->Gravity().Length() > 0 ? world_->Gravity().Normalized() : Vector3d(0, 0, -1);
    Vector3d w = down - u * down.Dot(u);
    if (w.Length() < 1e-6)
      w = u.Perpendicular();
    w.Normalize();
    std::vector<Vector3d> points;
    if (chord >= free_length_ * 0.999)   // taut: a straight line
      for (int i = 0; i <= n_; ++i)
        points.push_back(a + u * (chord * i / n_));
    else
    {
      const double theta = arcAngle(chord, free_length_), r = free_length_ / theta;
      // plane coordinates: x along u from the chord's middle, y along w (the sag side); the arc bulges to +y, its
      // centre at (0, yc) and its lowest point (0, yc + r) at the angle pi/2
      const double yc = -r * std::cos(theta / 2);
      const double a0 = std::atan2(-yc, -chord / 2), a1 = std::atan2(-yc, chord / 2);
      double sweep = a1 - a0;   // from A over pi/2 to B: decreasing angle, |sweep| = theta
      if (sweep > 0)
        sweep -= 2 * M_PI;
      const Vector3d mid = (a + b) * 0.5;
      for (int i = 0; i <= n_; ++i)
      {
        const double phi = a0 + sweep * i / n_;
        points.push_back(mid + u * (r * std::cos(phi)) + w * (yc + r * std::sin(phi)));
      }
    }
    std::vector<Pose3d> poses;
    const Vector3d normal = u.Cross(w).Normalized();
    for (int i = 0; i < n_; ++i)
    {
      Vector3d x = (points[i + 1] - points[i]).Normalized();
      Vector3d y = normal.Cross(x).Cross(x) * -1.0;
      if (y.Length() < 1e-6)
        y = x.Perpendicular();
      y.Normalize();
      Vector3d z = x.Cross(y);
      Matrix3d rot(x.X(), y.X(), z.X(), x.Y(), y.Y(), z.Y(), x.Z(), y.Z(), z.Z());
      poses.emplace_back((points[i] + points[i + 1]) * 0.5, Quaterniond(rot));
    }
    return poses;
  }

  static constexpr double kWideLimit = 1.5;   // [rad] the widest limit a joint starts with (a universal joint at 90 deg locks)

  // per joint: bend_limit, or the joint's bend in `poses` with a margin when that is more
  std::vector<double> startLimits(const std::vector<Pose3d> &poses) const
  {
    std::vector<double> limits{0.0};   // index 0 unused (joints are j1..j(n-1))
    for (int i = 1; i < n_; ++i)
    {
      const double c = poses[i - 1].Rot().XAxis().Dot(poses[i].Rot().XAxis());
      const double bend = std::acos(std::min(1.0, std::max(-1.0, c)));
      limits.push_back(std::min(kWideLimit, std::max(bend_limit_, 1.15 * bend + 0.02)));
    }
    return limits;
  }

  // the chain is spawned STRAIGHT (along x, 20 m below the world origin, out of everyone's way): Gazebo measures joint
  // angles from the spawn shape, so the bend limits and the bending springs then refer to the straight chain. At
  // attach it is moved into its hanging shape (rehang) before the clamps hold it. Its spawn limits are wide (the
  // hanging shape may bend more than bend_limit); rampLimits narrows them
  std::string chainSdf() const
  {
    std::vector<Pose3d> poses;
    for (int i = 0; i < n_; ++i)
      poses.emplace_back(Vector3d(link_length_ * (i + 0.5), 0, -20), Quaterniond::Identity);
    start_limits_.assign(n_, kWideLimit);
    const double m = mass_, s = inertia_scale_, r = radius_, h = link_length_;
    const double ixx = s * 0.5 * m * r * r, iyy = s * m * (3 * r * r + h * h) / 12.0;
    std::ostringstream out;
    out << "<sdf version='1.6'><model name='" << name_ << "'><self_collide>true</self_collide>";
    for (int i = 0; i < n_; ++i)
    {
      const Pose3d &p = poses[i];
      const Vector3d e = p.Rot().Euler();
      out << "<link name='l" << i << "'><pose>" << p.Pos().X() << " " << p.Pos().Y() << " " << p.Pos().Z() << " "
          << e.X() << " " << e.Y() << " " << e.Z() << "</pose><inertial><mass>" << m << "</mass><inertia><ixx>" << ixx
          << "</ixx><iyy>" << iyy << "</iyy><izz>" << iyy << "</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
          << "</inertial><collision name='c'><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder><radius>" << r
          << "</radius><length>" << std::max(0.001, h - 2 * r) << "</length></cylinder></geometry><surface><contact>"
          << "<collide_bitmask>" << bitmask_ << "</collide_bitmask></contact><friction><ode><mu>" << friction_
          << "</mu><mu2>" << friction_ << "</mu2></ode></friction></surface></collision><visual name='v'><pose>0 0 0 "
          << "0 1.5708 0</pose><geometry><cylinder><radius>" << r << "</radius><length>" << h << "</length></cylinder>"
          << "</geometry></visual></link>";
    }
    for (int i = 1; i < n_; ++i)
      out << "<joint name='j" << i << "' type='universal'><parent>l" << i - 1 << "</parent><child>l" << i
          << "</child><pose>" << -h / 2 << " 0 0 0 0 0</pose><axis><xyz>0 1 0</xyz><limit><lower>" << -start_limits_[i]
          << "</lower><upper>" << start_limits_[i] << "</upper></limit><dynamics><damping>" << damping_
          << "</damping>" << spring() << "</dynamics></axis><axis2><xyz>0 0 1</xyz><limit><lower>" << -start_limits_[i]
          << "</lower><upper>" << start_limits_[i] << "</upper></limit><dynamics><damping>" << damping_ << "</damping>"
          << spring() << "</dynamics></axis2><physics><ode><implicit_spring_damper>true</implicit_spring_damper>"
          << "</ode></physics></joint>";
    out << "</model></sdf>";
    return out.str();
  }

  std::string spring() const
  {
    if (bending_stiffness_ <= 0)
      return "";
    std::ostringstream out;
    out << "<spring_reference>0</spring_reference><spring_stiffness>" << bending_stiffness_ / link_length_
        << "</spring_stiffness>";
    return out.str();
  }

  // every collision of the host model loses the chain's bit, except the listed ones (name or link/collision)
  // the collide and category bits of ONE collision: Gazebo's ODECollision::SetCollideBits/SetCategoryBits also write
  // them to the link's collision space, so a chain-only bit on one added shape made the whole link (e.g. the robot's
  // base) collide with nothing but the chain and the Mobipick fell through the floor (#263, 2026-10-02); the ODE geom
  // alone is set here
  static void setBits(const physics::CollisionPtr &collision, unsigned int bits)
  {
    collision->GetSurface()->collideBitmask = bits;
    auto ode = boost::dynamic_pointer_cast<physics::ODECollision>(collision);
    if (ode && ode->GetCollisionId())
    {
      dGeomSetCollideBits(ode->GetCollisionId(), bits);
      dGeomSetCategoryBits(ode->GetCollisionId(), bits);
      return;
    }
    collision->SetCollideBits(bits);
    collision->SetCategoryBits(bits);
  }

  void restrictHostCollisions(const std::vector<std::string> &keep)
  {
    for (const auto &link : model_->GetLinks())
      for (const auto &collision : link->GetCollisions())
      {
        bool listed = false;   // a listed token anywhere in the collision's name (Gazebo renames lumped collisions)
        for (const auto &k : keep)
          listed = listed || collision->GetName().find(k) != std::string::npos ||
                   k == link->GetName() + "/" + collision->GetName();
        unsigned int bits = collision->GetSurface()->collideBitmask;
        bits = listed ? bitmask_ : (bits & ~bitmask_);   // listed: the chain's shapes, they touch only the chain
        setBits(collision, bits);
      }
  }

  // exclusive: every collision of every other model (tables, objects, also ones spawned later) loses the chain's bit,
  // so the chain and the listed host collisions touch nothing else
  void restrictOtherModels()
  {
    if (!exclusive_ || world_->ModelCount() == seen_models_)
      return;
    seen_models_ = world_->ModelCount();
    for (const auto &model : world_->Models())
    {
      if (model == model_ || model->GetName() == name_)
        continue;
      for (const auto &link : model->GetLinks())
        for (const auto &collision : link->GetCollisions())
        {
          setBits(collision, collision->GetSurface()->collideBitmask & ~bitmask_);
        }
    }
  }

  physics::JointPtr clampJoint(physics::LinkPtr host, physics::LinkPtr link, double x)
  {
    physics::JointPtr joint = world_->Physics()->CreateJoint("revolute", model_);
    joint->Attach(host, link);
    joint->Load(host, link, Pose3d(x, 0, 0, 0, 0, 0));
    joint->SetAxis(0, Vector3d(0, 0, 1));
    joint->SetUpperLimit(0, 0.0);
    joint->SetLowerLimit(0, 0.0);
    joint->SetProvideFeedback(true);
    joint->Init();
    return joint;
  }

  // once the host model exists: its links, the collision bits, the shapes, the chain model
  bool setup()
  {
    model_ = host_name_.empty() ? own_ : world_->ModelByName(host_name_);
    if (!model_)
      return false;
    parent_ = model_->GetLink(parent_name_);
    child_ = child_name_.empty() ? nullptr : model_->GetLink(child_name_);
    reference_ = reference_name_.empty() ? nullptr : model_->GetLink(reference_name_);
    if (!parent_ || (!child_name_.empty() && !child_))
    {
      gzerr << "gazebo_jointed_chain: parent_link / child_link not found in " << model_->GetName() << "\n";
      failed_ = true;
      return false;
    }
    restrictHostCollisions(keep_);
    for (const auto &shape : shapes_)
      addShape(shape);
    world_->InsertModelString(chainSdf());
    gzmsg << "gazebo_jointed_chain: " << name_ << " set up on " << model_->GetName()
          << (initial_points_.empty() ? "" : initialPoints().empty() ? ", initial shape NOT used (its ends are more than "
                                                                       "2 cm from the clamps): arc below the chord"
                                                                     : ", from the initial shape (" + 
                                                                       initialShapeText() + ")")
          << "\n";
    return true;
  }

  struct Shape
  {
    std::string name, link, type;
    Pose3d pose;
    std::string size;
  };

  // a collision on a host link at runtime, colliding only with the chain (ODE: attached to the link's body)
  void addShape(const Shape &shape)
  {
    physics::LinkPtr link = model_->GetLink(shape.link);
    if (!link)
    {
      gzerr << "gazebo_jointed_chain: shape " << shape.name << ": no link " << shape.link << "\n";
      return;
    }
    std::istringstream in(shape.size);
    double a = 0, b = 0, c = 0;
    in >> a >> b >> c;
    std::ostringstream geometry;
    if (shape.type == "box")
      geometry << "<box><size>" << a << " " << b << " " << c << "</size></box>";
    else if (shape.type == "cylinder")
      geometry << "<cylinder><radius>" << a << "</radius><length>" << b << "</length></cylinder>";
    else
      geometry << "<sphere><radius>" << a << "</radius></sphere>";
    const Vector3d e = shape.pose.Rot().Euler();
    std::ostringstream xml;
    xml << "<sdf version='1.6'><model name='m'><link name='l'><collision name='" << shape.name << "'><pose>"
        << shape.pose.Pos().X() << " " << shape.pose.Pos().Y() << " " << shape.pose.Pos().Z() << " " << e.X() << " "
        << e.Y() << " " << e.Z() << "</pose><geometry>" << geometry.str() << "</geometry><surface><contact>"
        << "<collide_bitmask>" << bitmask_ << "</collide_bitmask></contact><friction><ode><mu>" << friction_
        << "</mu><mu2>" << friction_ << "</mu2></ode></friction></surface></collision></link></model></sdf>";
    sdf::SDFPtr doc(new sdf::SDF());
    sdf::init(doc);
    if (!sdf::readString(xml.str(), doc))
    {
      gzerr << "gazebo_jointed_chain: shape " << shape.name << ": bad SDF\n";
      return;
    }
    sdf::ElementPtr elem = doc->Root()->GetElement("model")->GetElement("link")->GetElement("collision");
    physics::CollisionPtr collision = world_->Physics()->CreateCollision(shape.type, link);
    collision->Load(elem);
    collision->Init();
    link->AddChild(collision);
    auto ode = boost::dynamic_pointer_cast<physics::ODECollision>(collision);
    auto odeLink = boost::dynamic_pointer_cast<physics::ODELink>(link);
    if (ode && odeLink && ode->GetCollisionId() && odeLink->GetODEId())
    {
      dGeomSetBody(ode->GetCollisionId(), odeLink->GetODEId());
      ode->OnPoseChange();
    }
    setBits(collision, bitmask_);
  }

  void onUpdate()
  {
    if (!model_)
    {
      if (failed_ || !setup())
        return;
    }
    if (!chain_)
    {
      chain_ = world_->ModelByName(name_);
      if (!chain_)
        return;
      for (int i = 0; i < n_; ++i)
        links_.push_back(chain_->GetLink("l" + std::to_string(i)));
      rehang();   // from the straight spawn into the hanging (or initial) shape, before the clamps hold it
      clamp_a_ = clampJoint(parent_, links_.front(), -link_length_ / 2);
      if (child_)
        clamp_b_ = clampJoint(child_, links_.back(), link_length_ / 2);
      last_publish_ = world_->SimTime();
      const Pose3d l0 = links_.front()->WorldPose(), ln = links_.back()->WorldPose();
      gzmsg << "gazebo_jointed_chain: " << name_ << " attached; clamp A " << clampA() << ", link 0 start "
            << l0.Pos() + l0.Rot().RotateVector(Vector3d(-link_length_ / 2, 0, 0)) << "; clamp B " << clampB()
            << ", last link end " << ln.Pos() + ln.Rot().RotateVector(Vector3d(link_length_ / 2, 0, 0)) << "\n";
    }
    restrictOtherModels();
    if (reset_requested_)
    {
      reset_requested_ = false;
      rehang();
    }
    rampLimits();
    const common::Time now = world_->SimTime();
    if (rate_ <= 0 || (now - last_publish_).Double() < 1.0 / rate_)
      return;
    last_publish_ = now;
    publish();
  }

  // the joint limits from start_limits_ to bend_limit_ over settle_time_ (linear in sim time)
  void rampLimits()
  {
    if (!ramping_)
    {
      if (!ramp_pending_ || start_limits_.empty())
        return;
      ramp_pending_ = false;
      ramp_start_ = world_->SimTime();
      ramping_ = true;
    }
    const double f = settle_time_ > 0 ? std::min(1.0, (world_->SimTime() - ramp_start_).Double() / settle_time_) : 1.0;
    for (int i = 1; i < n_; ++i)
    {
      physics::JointPtr joint = chain_->GetJoint("j" + std::to_string(i));
      if (!joint)
        continue;
      const double limit = start_limits_[i] + (bend_limit_ - start_limits_[i]) * f;
      for (unsigned int axis = 0; axis < 2; ++axis)
      {
        joint->SetUpperLimit(axis, limit);
        joint->SetLowerLimit(axis, -limit);
      }
    }
    if (f >= 1.0)
    {
      ramping_ = false;
      over_ticks_ = 0;
    }
  }

  void rehang()
  {
    const std::vector<Pose3d> poses = arcPoses();
    start_limits_ = startLimits(poses);
    ramp_pending_ = true;
    ramping_ = false;
    for (int i = 0; i < n_; ++i)
    {
      links_[i]->SetWorldPose(poses[i]);
      links_[i]->ResetPhysicsStates();
    }
  }

  void publish()
  {
    const double fa = clamp_a_->GetForceTorque(0).body2Force.Length();
    const double fb = clamp_b_ ? clamp_b_->GetForceTorque(0).body2Force.Length() : 0.0;
    const Pose3d ref = reference_ ? reference_offset_ + reference_->WorldPose() : Pose3d::Zero;
    std::vector<Vector3d> points{clampA()};
    for (const auto &link : links_)
    {
      const Pose3d p = link->WorldPose();
      points.push_back(p.Pos() + p.Rot().RotateVector(Vector3d(-link_length_ / 2, 0, 0)));
      points.push_back(p.Pos() + p.Rot().RotateVector(Vector3d(link_length_ / 2, 0, 0)));
    }
    if (child_)
      points.push_back(clampB());
    double length = 0;
    for (size_t i = 1; i < points.size(); ++i)
      length += points[i].Distance(points[i - 1]);
    const double strain = length / free_length_ - 1.0;
    std_msgs::Float64MultiArray msg;
    msg.data = {fa, fb, strain, static_cast<double>(points.size())};
    for (const auto &p : points)
    {
      const Vector3d q = ref.Rot().RotateVectorReverse(p - ref.Pos());
      msg.data.insert(msg.data.end(), {q.X(), q.Y(), q.Z()});
    }
    if (state_pub_)
      state_pub_.publish(msg);
    const bool over = (stop_strain_ > 0 && strain > stop_strain_) || (stop_force_ > 0 && std::max(fa, fb) > stop_force_);
    over_ticks_ = over && !ramping_ ? over_ticks_ + 1 : 0;
    if (over_ticks_ >= stop_ticks_ && !stopped_ && stop_client_)
    {
      stopped_ = true;
      gzwarn << "gazebo_jointed_chain: " << name_ << " strain " << strain << ", clamp forces " << fa << " / " << fb
             << " N: calling " << stop_service_ << "\n";
      std::thread([client = stop_client_]() mutable {
        std_srvs::SetBool call;
        call.request.data = true;
        client.call(call);
      }).detach();
    }
    if (!over)
      stopped_ = false;
  }

  bool onReset(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    reset_requested_ = true;   // applied in the physics thread at the next update
    return true;
  }

  physics::ModelPtr model_, own_, chain_;
  std::string host_name_, parent_name_, child_name_, reference_name_;
  std::vector<std::string> keep_;
  std::vector<Shape> shapes_;
  bool failed_ = false;
  physics::WorldPtr world_;
  physics::LinkPtr parent_, child_, reference_;
  std::vector<physics::LinkPtr> links_;
  physics::JointPtr clamp_a_, clamp_b_;
  Pose3d parent_offset_, child_offset_;
  std::string name_, stop_service_;
  int n_ = 15, stop_ticks_ = 3, over_ticks_ = 0;
  double free_length_ = 0.52, link_length_ = 0.0, radius_ = 0.0175, mass_ = 0.0122, inertia_scale_ = 4.0;
  double bending_stiffness_ = 0.0;
  double bend_limit_ = 0.35, damping_ = 0.002, friction_ = 0.3, rate_ = 10.0, stop_strain_ = 0.0, stop_force_ = 0.0;
  unsigned int bitmask_ = 0x8000;
  std::vector<Vector3d> initial_points_;   // parent link frame
  Pose3d reference_offset_;                 // the published frame in reference_link (a link Gazebo merged away)
  double settle_time_ = 2.0;
  mutable std::vector<double> start_limits_;   // per joint j1..j(n-1), from the spawn shape
  common::Time ramp_start_;
  bool ramp_pending_ = true, ramping_ = false;
  bool stopped_ = false, exclusive_ = true;
  unsigned int seen_models_ = 0;
  bool reset_requested_ = false;
  common::Time last_publish_;
  event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> ros_;
  ros::Publisher state_pub_;
  ros::ServiceServer reset_srv_;
  ros::ServiceClient stop_client_;
};

GZ_REGISTER_MODEL_PLUGIN(JointedChainPlugin)
}  // namespace gazebo
