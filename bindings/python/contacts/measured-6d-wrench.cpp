#include "tsid/bindings/python/contacts/measured-6d-wrench.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"

namespace tsid {
namespace python {
void exposeMeasured6dWrench() {
  Measured6dWrenchPythonVisitor<tsid::contacts::Measured6Dwrench>::expose(
      "Measured6dWrench");
}
}  // namespace python
}  // namespace tsid