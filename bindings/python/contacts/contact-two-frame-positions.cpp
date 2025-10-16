//
// Copyright (c) 2023 MIPT
//

#include "tsid/bindings/python/contacts/contact-two-frame-positions.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"

namespace tsid {
namespace python {
void exposeContactTwoFramePositions() {
  ContactTwoFramePositionsPythonVisitor<
      tsid::contacts::ContactTwoFramePositions>::
      expose("ContactTwoFramePositions");
}
}  // namespace python
}  // namespace tsid
