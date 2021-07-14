/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_TYPE_TYPE_VEC_H
#define OV_TYPE_TYPE_VEC_H

#include "Type.h"

namespace ov_type {

/**
 * @brief Derived Type class that implements vector variables
 */
class Vec : public Type {

public:
  /**
   * @brief Default constructor for Vec
   * @param dim Size of the vector (will be same as error state)
   */
  Vec(int dim) : Type(dim) {
    _value = Eigen::VectorXd::Zero(dim);
    _fej = Eigen::VectorXd::Zero(dim);
  }

  ~Vec() {}

  /**
   * @brief Implements the update operation through standard vector addition
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd &dx) override {
    assert(dx.rows() == _size);
    set_value(_value + dx);
  }

  /**
   * @brief Performs all the cloning
   */
  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<Type>(new Vec(_size));
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_VEC_H