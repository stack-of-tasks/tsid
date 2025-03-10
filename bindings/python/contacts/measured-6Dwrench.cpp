#include "tsid/bindings/python/contacts/measured-6Dwrench.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"

namespace tsid {
namespace python {
void exposeMeasured6Dwrench() {
  Measured6DWrenchPythonVisitor<tsid::contacts::Measured6Dwrench>::expose(
      "Measured6Dwrench");
}
}  // namespace python
}  // namespace tsid