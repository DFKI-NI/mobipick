#ifndef MOBIPICK_PICK_N_PLACE_FAKE_OBJECT_RECOGNITION_H
#define MOBIPICK_PICK_N_PLACE_FAKE_OBJECT_RECOGNITION_H

#include <iostream>

enum ObjectID
{
  GETRIEBELAGER = 1,
  POWER_DRILL = 2,
  TABLE = 100,
  COKE_CAN = 101,
  ROOF = 10
};

#endif  // MOBIPICK_PICK_N_PLACE_FAKE_OBJECT_RECOGNITION_H

std::string id_to_string(int id)
{
  switch (id)
  {
    case ObjectID::GETRIEBELAGER:
      return "getriebelager";
    case ObjectID::POWER_DRILL:
      return "power_drill";
    case ObjectID::TABLE:
      return "table";
    case ObjectID::COKE_CAN:
      return "coke_can";
    case ObjectID::ROOF:
      return "roof";
    default:
      std::cerr << "No such ObjectID: " << id << std::endl;
      return "";
  }
}
