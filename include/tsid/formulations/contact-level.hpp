//
// Copyright (c) 2021 University of Trento
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __tsid_contact_level_hpp__
#define __tsid_contact_level_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/contacts/contact-base.hpp"

namespace tsid
{

  /** Data structure collecting information regarding a single contact.
   * In particular, this structure contains the index of the force corresponding
   * to this contact in the force vector used as decision variable in the QP.
   * Moreover it contains all the default constraints associated to a contact for representing
   * the motion constraints (contact points do not move), the friction cone constraints
   * and the force regularization cost.
   */
  struct ContactLevel
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    contacts::ContactBase & contact;
    std::shared_ptr<math::ConstraintBase> motionConstraint;
    std::shared_ptr<math::ConstraintInequality> forceConstraint;
    std::shared_ptr<math::ConstraintEquality> forceRegTask;
    unsigned int index; /// index of 1st element of associated force variable in the force vector

    ContactLevel(contacts::ContactBase & contact);
  };

}

#endif // ifndef __tsid_contact_level_hpp__
