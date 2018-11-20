#include "tsid/bindings/python/contacts/contact.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"

namespace tsid
{
  namespace python
  {
    void exposeContact6d()
    {
      ContactPythonVisitor<tsid::contacts::Contact6d>::expose("Contact6d");
    }
  }
}
