#ifndef __tsid_python_expose_contact_hpp__
#define __tsid_python_expose_contact_hpp__

#include "tsid/bindings/python/contacts/contact.hpp"

namespace tsid
{
  namespace python
  {
    void exposeContact6d();
    inline void exposeContact()
    {
      exposeContact6d();

    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_contact_hpp__
