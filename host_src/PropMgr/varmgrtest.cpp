#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

#include "PropertyMgr.h"
#include "OnChangeObj.h"

using namespace std;

class testcallback : public OnChangeObj
{
   public:
	  void onChange(PropertyValue * _pv)
	  {
		 cout << "                         " << "testcallback received onChange notification" << endl;
	  }
};

int main()
{
   {
	  testcallback tcb;
	  string p("/a/bb/ccc/dddd/eeeee/ffffff");
	  PropertyMgr mgr;
	  cout << " print #1" << endl;
	  mgr.print();
	  
	  PropertyValue * p1 = mgr.addLeaf("/test/wholenumber");
	  p1->set(17);
	  PropertyValue * p2;
	  (p2 = mgr.addLeaf("/lesser/floatno"))->set(-0.334f);
	  PropertyValue * p3;
	  (p3 = mgr.addLeaf("/dict/text/str"))->set(string("laber"));
	  //PropertyValue * nd =
	                        mgr.addLeaf("/i/am/notdefined");
	  PropertyValue * p4;
	  (p4 = mgr.addLeaf("/logical/notlogic/MisterBoole"))->set(true);
	  cout << " print #2" << endl;
	  mgr.print();
	  p1->registerOnChange(&tcb);
	  p1->set(99);
	  
	  cout << " print #3" << endl;
	  mgr.print();
//	  PropertyValue * r0 = 
                            mgr.get("/this/does/not/exist");
	  PropertyValue * r1 = mgr.get("/test/wholenumber");
	  PropertyValue * notdef = mgr.get("/i/am/notdefined");
	  notdef->set(false);
	  cout << "getPath test: " << p1->getPath() << endl;
	  cout << "\\\\\\\\\\\\\\" << "comparing p1 and r1: p1=" << &p1 << "  r1=" << r1 << endl;
	  r1->set(23);
	  PropertyValue * r3 = mgr.get("/dict/text/str");
	  r3->set(string("unit TESTING in progress..."));
	  
	  mgr.run();

	  cout << " print #4" << endl;
	  mgr.print();
   }
   
   exit(0);
}
