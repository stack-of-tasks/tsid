//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/contacts/contact-point.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"

namespace tsid {
namespace python {
void exposeContactPoint() {
  ContactPointPythonVisitor<tsid::contacts::ContactPoint>::expose(
      "ContactPoint");
}
}  // namespace python
}  // namespace tsid
