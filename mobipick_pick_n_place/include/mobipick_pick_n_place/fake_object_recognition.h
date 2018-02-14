#ifndef MOBIPICK_PICK_N_PLACE_FAKE_OBJECT_RECOGNITION_H
#define MOBIPICK_PICK_N_PLACE_FAKE_OBJECT_RECOGNITION_H

#include <iostream>

enum ObjectID
{
  TABLE = 100, COKE_CAN = 101
};

#endif //MOBIPICK_PICK_N_PLACE_FAKE_OBJECT_RECOGNITION_H

std::string id_to_string(int id)
{
  switch (id)
  {
    case ObjectID::TABLE:
      return "table";
    case ObjectID::COKE_CAN:
      return "coke_can";
    default:
      std::cerr << "No such ObjectID: " << id << std::endl;
      return "";
  }
}