// Carrier link chain of the arm cable model: the solver of CarrierLinkChains in cable_model.py.
//
// One call advances all chains by one model update (dt) in many small substeps: position based dynamics on the
// joints of the chain (point masses), with
//   - link lengths and joint stops projected one after the other along the chain, forwards and backwards;
//   - the joint stop as the distance of a joint from the middle between its two neighbours (it grows with the turn);
//   - capsules (arm and tool, already widened by the disc radius) that glide from their last place to the new one,
// tapered towards clamps that sit inside them, with Coulomb friction against the moving capsule surface;
//   - the carrier not passing through itself (joints far enough apart along the chain keep one disc diameter).
// Plain C interface on the caller's arrays (numpy, row-major doubles), no state kept here.
//
// Build: catkin (CMakeLists.txt), or `g++ -O2 -shared -fPIC disc_chain.cpp -o libdisc_chain.so` (disc_native.py does
// that on its own when no built library is found).
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

struct V3 {
  double x, y, z;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 lerp(V3 a, V3 b, double t) { return a + (b - a) * t; }
inline V3 get(const double* p, int i) { return {p[3 * i], p[3 * i + 1], p[3 * i + 2]}; }
inline void put(double* p, int i, V3 v) {
  p[3 * i] = v.x;
  p[3 * i + 1] = v.y;
  p[3 * i + 2] = v.z;
}

struct Capsule {
  V3 a, b;
  double r;       // sweep radius
  double core;    // core radius: 0 = a capsule, > 0 = a flat cylinder with edges rounded by r
  double half[3] = {0, 0, 0};   // > 0 = a rounded box: core half sizes, centre a
  double rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};   // its axes (columns, row-major)
};

// one shape from its 20 numbers (a, b, sweep radius, core radius, box half sizes, box rotation)
inline Capsule shape_at(const double* s) {
  Capsule c;
  c.a = V3{s[0], s[1], s[2]};
  c.b = V3{s[3], s[4], s[5]};
  c.r = s[6];
  c.core = s[7];
  for (int k = 0; k < 3; ++k) c.half[k] = s[8 + k];
  for (int k = 0; k < 9; ++k) c.rot[k] = s[11 + k];
  return c;
}

// signed distance of p from the capsule's core (its axis, or for a flat cylinder the disc of radius `core` along
// it) and the unit normal pointing out; negative inside the core
inline double core_distance(const Capsule& c, V3 p, V3* normal) {
  if (c.half[0] > 0.0) {   // a rounded box
    const double* R = c.rot;
    V3 d = p - c.a;
    double l[3] = {R[0] * d.x + R[3] * d.y + R[6] * d.z, R[1] * d.x + R[4] * d.y + R[7] * d.z,
                   R[2] * d.x + R[5] * d.y + R[8] * d.z};
    double q[3], g[3];
    bool inside = true;
    for (int k = 0; k < 3; ++k) {
      q[k] = std::min(c.half[k], std::max(-c.half[k], l[k]));
      g[k] = c.half[k] - std::fabs(l[k]);
      inside = inside && g[k] > 0.0;
    }
    if (inside) {   // out through the nearest face
      int ax = g[0] <= g[1] ? (g[0] <= g[2] ? 0 : 2) : (g[1] <= g[2] ? 1 : 2);
      const double s = l[ax] < 0.0 ? -1.0 : 1.0;
      *normal = V3{R[ax] * s, R[3 + ax] * s, R[6 + ax] * s};
      return -g[ax];
    }
    V3 off{l[0] - q[0], l[1] - q[1], l[2] - q[2]};   // in box axes
    V3 w{R[0] * off.x + R[1] * off.y + R[2] * off.z, R[3] * off.x + R[4] * off.y + R[5] * off.z,
         R[6] * off.x + R[7] * off.y + R[8] * off.z};
    const double dist = norm(w);
    *normal = dist > 1e-9 ? w * (1.0 / dist) : V3{0, 0, 1};
    return dist;
  }
  V3 ab = c.b - c.a;
  double len2 = dot(ab, ab);
  double t_raw = len2 > 1e-12 ? dot(p - c.a, ab) / len2 : 0.0;
  double t = std::min(1.0, std::max(0.0, t_raw));
  V3 axial = c.a + ab * t;
  V3 offset = p - axial;
  if (c.core > 0.0) {
    const double len = std::sqrt(len2);
    V3 u = len > 1e-12 ? ab * (1.0 / len) : V3{0, 0, 1};
    V3 w = offset - u * dot(offset, u);
    const double rho = norm(w);
    V3 w_hat = rho > 1e-12 ? w * (1.0 / rho) : V3{0, 0, 0};
    if (rho < c.core && t_raw > 0.0 && t_raw < 1.0) {   // inside the core: out through the nearest surface
      const double side = c.core - rho, to_a = t_raw * len, to_b = (1.0 - t_raw) * len;
      if (side <= to_a && side <= to_b) { *normal = w_hat; return -side; }
      if (to_a <= to_b) { *normal = u * -1.0; return -to_a; }
      *normal = u;
      return -to_b;
    }
    offset = offset - w_hat * std::min(rho, c.core);
  }
  const double dist = norm(offset);
  *normal = dist > 1e-9 ? offset * (1.0 / dist) : V3{0, 0, 1};
  return dist;
}

}  // namespace

extern "C" {

// the argument layout; disc_native.ABI_VERSION must match (a library built from older source is skipped)
int disc_chain_abi_version() { return 4; }

struct DiscChainParams {
  int substeps;          // per call
  int passes;            // constraint passes per substep
  double gravity;        // [m/s^2]
  double keep;           // share of the velocity kept per substep (damping)
  double turn;           // joint stop [rad]
  double self_distance;  // joints at least self_skip links apart keep this distance [m]; 0 = off
  double clamp_taper;    // capsule radius grows by this per metre of chain away from a clamp inside it; 0 = off
  double friction;       // Coulomb coefficient of the carrier on the capsules; 0 = none
};

// x, x_prev: n_nodes x 3 (in/out). inv_mass: n_nodes (0 = pinned). Chain c: nodes chain_start[c] ..
// chain_start[c] + chain_n[c] - 1, link length chain_seg[c], self collision from chain_self_skip[c] links apart
// (0 = none). pins0/pins1: n_chains x 12 (clamp a, its direction, clamp b, its direction) at the last update and now.
// caps0/caps1: n_caps x 20 (core ends a, b, sweep radius, core radius, box half sizes, box rotation; ). cap_motion: n_caps x 12 (rotation 3x3 row-major, translation)
// taking a point fixed to the capsule's link from its place at the last update to its place now, or null.
// strain: n_chains (out), chain length over its nominal length minus one. Returns 0, or 1 when a value is not finite.
int disc_chain_step(double* x, double* x_prev, const double* inv_mass, int n_nodes, const int* chain_start,
                    const int* chain_n, const double* chain_seg, const int* chain_self_skip, int n_chains,
                    const double* pins0, const double* pins1, const double* caps0, const double* caps1,
                    const double* cap_motion, int n_caps, const double* down, double dt,
                    const DiscChainParams* prm, double* strain) {
  const int substeps = std::max(1, prm->substeps);
  const double h = dt / substeps;
  const V3 fall = V3{down[0], down[1], down[2]} * (prm->gravity * h * h);
  const double stop = std::sin(prm->turn / 2.0);
  std::vector<Capsule> caps(n_caps);
  std::vector<double> clamp_dist(2 * n_caps);
  std::vector<int> near;
  near.reserve(n_caps);

  for (int k = 0; k < substeps; ++k) {
    const double alpha = (k + 1.0) / substeps;
    for (int m = 0; m < n_caps; ++m) {
      caps[m] = shape_at(caps1 + 20 * m);   // box half sizes and rotation: the new ones (a rotation cannot be lerped)
      caps[m].a = lerp(get(caps0 + 20 * m, 0), get(caps1 + 20 * m, 0), alpha);
      caps[m].b = lerp(get(caps0 + 20 * m, 1), get(caps1 + 20 * m, 1), alpha);
      caps[m].r = caps0[20 * m + 6] + (caps1[20 * m + 6] - caps0[20 * m + 6]) * alpha;
      caps[m].core = caps0[20 * m + 7] + (caps1[20 * m + 7] - caps0[20 * m + 7]) * alpha;
    }
    for (int i = 0; i < n_nodes; ++i) {
      V3 p = get(x, i);
      if (inv_mass[i] > 0.0) {
        V3 v = (p - get(x_prev, i)) * prm->keep;
        put(x_prev, i, p);
        put(x, i, p + v + fall);
      } else {
        put(x_prev, i, p);
      }
    }
    for (int c = 0; c < n_chains; ++c) {
      const int s0 = chain_start[c], n = chain_n[c];
      const double seg = chain_seg[c];
      V3 a = lerp(get(pins0 + 12 * c, 0), get(pins1 + 12 * c, 0), alpha);
      V3 ta = lerp(get(pins0 + 12 * c, 1), get(pins1 + 12 * c, 1), alpha);
      V3 b = lerp(get(pins0 + 12 * c, 2), get(pins1 + 12 * c, 2), alpha);
      V3 tb = lerp(get(pins0 + 12 * c, 3), get(pins1 + 12 * c, 3), alpha);
      ta = ta * (1.0 / std::max(norm(ta), 1e-9));
      tb = tb * (1.0 / std::max(norm(tb), 1e-9));
      put(x, s0, a);
      put(x, s0 + 1, a + ta * seg);
      put(x, s0 + n - 1, b);
      put(x, s0 + n - 2, b + tb * seg);
    }

    for (int pass = 0; pass < prm->passes; ++pass) {
      for (int c = 0; c < n_chains; ++c) {
        const int s0 = chain_start[c], n = chain_n[c];
        const double seg = chain_seg[c], limit = seg * stop;
        // link lengths, along the chain and back
        for (int dir = 0; dir < 2; ++dir) {
          for (int j = 0; j < n - 1; ++j) {
            const int i = s0 + (dir == 0 ? j : n - 2 - j);
            const double wi = inv_mass[i], wj = inv_mass[i + 1];
            if (wi + wj <= 0.0) continue;
            V3 pi = get(x, i), pj = get(x, i + 1), d = pj - pi;
            const double len = std::max(norm(d), 1e-12);
            V3 corr = d * ((len - seg) / len / (wi + wj));
            put(x, i, pi + corr * wi);
            put(x, i + 1, pj - corr * wj);
          }
        }
        // joint stops
        for (int dir = 0; dir < 2; ++dir) {
          for (int j = 0; j < n - 2; ++j) {
            const int i = s0 + (dir == 0 ? j : n - 3 - j);
            const double w0 = inv_mass[i], w1 = inv_mass[i + 1], w2 = inv_mass[i + 2];
            const double denom = w1 + 0.25 * (w0 + w2);
            if (denom <= 0.0) continue;
            V3 p0 = get(x, i), p1 = get(x, i + 1), p2 = get(x, i + 2);
            V3 d = p1 - (p0 + p2) * 0.5;
            const double off = norm(d);
            if (off <= limit) continue;
            V3 corr = d * ((off - limit) / off / denom);
            put(x, i + 1, p1 - corr * w1);
            put(x, i, p0 + corr * (0.5 * w0));
            put(x, i + 2, p2 + corr * (0.5 * w2));
          }
        }
        // the carrier does not pass through itself
        const int skip = chain_self_skip[c];
        if (prm->self_distance > 0.0 && skip > 0) {
          for (int i = s0; i < s0 + n - skip; ++i) {
            for (int j = i + skip; j < s0 + n; ++j) {
              const double wi = inv_mass[i], wj = inv_mass[j];
              if (wi + wj <= 0.0) continue;
              V3 pi = get(x, i), pj = get(x, j), d = pj - pi;
              const double len2 = dot(d, d);
              if (len2 >= prm->self_distance * prm->self_distance) continue;
              const double len = std::max(std::sqrt(len2), 1e-12);
              V3 corr = d * ((len - prm->self_distance) / len / (wi + wj));
              put(x, i, pi + corr * wi);
              put(x, j, pj - corr * wj);
            }
          }
        }
      }
      // capsules, with the taper towards clamps inside them and friction
      for (int c = 0; c < n_chains; ++c) {
        const int s0 = chain_start[c], n = chain_n[c];
        const double seg = chain_seg[c];
        V3 unused;
        // only the shapes within reach of this chain's box (a chain touches a few of the arm's shapes)
        V3 lo = get(x, s0), hi = lo;
        for (int j = 1; j < n; ++j) {
          V3 q = get(x, s0 + j);
          lo = V3{std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
          hi = V3{std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
        }
        near.clear();
        for (int m = 0; m < n_caps; ++m) {
          const double reach = caps[m].r + caps[m].core + 0.05 +
                               std::sqrt(caps[m].half[0] * caps[m].half[0] + caps[m].half[1] * caps[m].half[1] +
                                         caps[m].half[2] * caps[m].half[2]);
          const Capsule& cm = caps[m];
          if (std::min(cm.a.x, cm.b.x) - reach > hi.x || std::max(cm.a.x, cm.b.x) + reach < lo.x ||
              std::min(cm.a.y, cm.b.y) - reach > hi.y || std::max(cm.a.y, cm.b.y) + reach < lo.y ||
              std::min(cm.a.z, cm.b.z) - reach > hi.z || std::max(cm.a.z, cm.b.z) + reach < lo.z)
            continue;
          near.push_back(m);
        }
        for (int m : near) {
          clamp_dist[2 * m] = std::max(0.0, core_distance(caps[m], get(x, s0 + 1), &unused));
          clamp_dist[2 * m + 1] = std::max(0.0, core_distance(caps[m], get(x, s0 + n - 2), &unused));
        }
        for (int j = 2; j < n - 2; ++j) {
          const int i = s0 + j;
          V3 p = get(x, i);
          double worst = 0.0, worst_r = 0.0, worst_dist = 0.0;
          V3 worst_nrm{0, 0, 1};
          int hit = -1;
          for (int m : near) {
            V3 nrm_m;
            const double dist = core_distance(caps[m], p, &nrm_m);
            double r = caps[m].r;
            if (prm->clamp_taper > 0.0) {
              r = std::min(r, std::min(clamp_dist[2 * m] + prm->clamp_taper * (j - 1) * seg,
                                       clamp_dist[2 * m + 1] + prm->clamp_taper * (n - 2 - j) * seg));
            }
            if (dist - r < worst) {
              worst = dist - r;
              worst_r = r;
              worst_dist = dist;
              worst_nrm = nrm_m;
              hit = m;
            }
          }
          if (hit < 0) continue;
          V3 nrm = worst_nrm;
          p = p + nrm * (worst_r - worst_dist);
          if (prm->friction > 0.0) {
            // what the joint moved along the surface in this substep, against the surface itself
            V3 moved = p - get(x_prev, i);
            if (cap_motion) {
              const double* mo = cap_motion + 12 * hit;
              V3 q = get(x_prev, i);
              V3 carried{mo[0] * q.x + mo[1] * q.y + mo[2] * q.z + mo[9] - q.x,
                         mo[3] * q.x + mo[4] * q.y + mo[5] * q.z + mo[10] - q.y,
                         mo[6] * q.x + mo[7] * q.y + mo[8] * q.z + mo[11] - q.z};
              moved = moved - carried * (1.0 / substeps);
            }
            V3 slide = moved - nrm * dot(moved, nrm);
            const double s = norm(slide), grip = prm->friction * (worst_r - worst_dist);
            if (s > 1e-12) p = p - slide * std::min(1.0, grip / s);
          }
          put(x, i, p);
        }
      }
    }
  }

  int bad = 0;
  for (int c = 0; c < n_chains; ++c) {
    const int s0 = chain_start[c], n = chain_n[c];
    double length = 0.0;
    for (int i = s0; i < s0 + n - 1; ++i) length += norm(get(x, i + 1) - get(x, i));
    strain[c] = length / (chain_seg[c] * (n - 1)) - 1.0;
    if (!std::isfinite(strain[c])) bad = 1;
  }
  return bad;
}

// Ropes.tighten in C++ (the Python version called the closest-point search ~100 times per model update).
// x: n x 3 (in/out), a polyline with its two end nodes at each end fixed; caps: n_caps x 20 (see disc_chain_step). Red-black Gauss-Seidel: every free node moves to the middle of its neighbours, then out of
// the shape it is deepest in.
void rope_tighten(double* x, int n, const double* caps_in, int n_caps, int iterations) {
  std::vector<Capsule> caps(n_caps);
  for (int m = 0; m < n_caps; ++m) caps[m] = shape_at(caps_in + 20 * m);
  for (int it = 0; it < iterations; ++it) {
    for (int start = 2; start <= 3; ++start) {
      for (int i = start; i < n - 2; i += 2) {
        V3 p = (get(x, i - 1) + get(x, i + 1)) * 0.5;
        double worst = 0.0, worst_r = 0.0, worst_dist = 0.0;
        V3 worst_nrm{0, 0, 1};
        for (int m = 0; m < n_caps; ++m) {
          V3 nrm_m;
          const double dist = core_distance(caps[m], p, &nrm_m);
          if (dist - caps[m].r < worst) {
            worst = dist - caps[m].r;
            worst_r = caps[m].r;
            worst_dist = dist;
            worst_nrm = nrm_m;
          }
        }
        if (worst < 0.0) p = p + worst_nrm * (worst_r - worst_dist);
        put(x, i, p);
      }
    }
  }
}

// Ropes._collide in C++: every listed node out of the shape it is deepest in (x: nodes x 3, in/out)
void rope_collide(double* x, const int* idx, int n_idx, const double* caps_in, int n_caps) {
  std::vector<Capsule> caps(n_caps);
  for (int m = 0; m < n_caps; ++m) caps[m] = shape_at(caps_in + 20 * m);
  for (int k = 0; k < n_idx; ++k) {
    const int i = idx[k];
    V3 p = get(x, i);
    double worst = 0.0, worst_r = 0.0, worst_dist = 0.0;
    V3 worst_nrm{0, 0, 1};
    for (int m = 0; m < n_caps; ++m) {
      V3 nrm_m;
      const double dist = core_distance(caps[m], p, &nrm_m);
      if (dist - caps[m].r < worst) {
        worst = dist - caps[m].r;
        worst_r = caps[m].r;
        worst_dist = dist;
        worst_nrm = nrm_m;
      }
    }
    if (worst < 0.0) put(x, i, p + worst_nrm * (worst_r - worst_dist));
  }
}

int rope_abi_version() { return 4; }

}  // extern "C"
