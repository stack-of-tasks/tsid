//
// Copyright (c) 2021 University of Trento
//

#ifndef __tsid_contact_level_hpp__
#define __tsid_contact_level_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/contacts/contact-base.hpp"

namespace tsid {

/** Data structure collecting information regarding a single contact.
 * In particular, this structure contains the index of the force corresponding
 * to this contact in the force vector used as decision variable in the QP.
 * Moreover it contains all the default constraints associated to a contact for
 * representing the motion constraints (contact points do not move), the
 * friction cone constraints and the force regularization cost.
 */
struct ContactLevel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  contacts::ContactBase& contact;
  std::shared_ptr<math::ConstraintBase> motionConstraint;
  std::shared_ptr<math::ConstraintInequality> forceConstraint;
  std::shared_ptr<math::ConstraintEquality> forceRegTask;
  unsigned int index;  /// index of 1st element of associated force variable in
                       /// the force vector

  ContactLevel(contacts::ContactBase& contact);
};

}  // namespace tsid

#endif  // ifndef __tsid_contact_level_hpp__
