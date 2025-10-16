//
// Copyright (c) 2018 CNRS, NYU, MPI Tübingen
//

#include "tsid/bindings/python/contacts/contact-6d.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"

namespace tsid {
namespace python {
void exposeContact6d() {
  Contact6DPythonVisitor<tsid::contacts::Contact6d>::expose("Contact6d");
}
}  // namespace python
}  // namespace tsid
