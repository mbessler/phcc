//PropertyLeaf.cpp - The parts of PropertyMgr that cannot have children in the tree
//
// This file is part of X11GC.
//
//  X11GC is a Glass Cockpit Software Suite for X11,
//  which does NOT use OpenGL but relies only on xlib.
//  Copyright (C) 2003 Manuel Bessler
//
//  The full text of the legal notices is contained in the file called
//  COPYING, included with this distribution.
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// This is for emacs
// Local variables:
// mode: c++
// c-basic-offset: 3
// tab-width: 3
// End:
//
//  $Id: $
////////////////////////////////////////////////////////////////////////

#include "PropertyLeaf.h"
#include "PropertyValue.h"
#include "PropertyBranch.h"

using namespace std;


PropertyLeaf::PropertyLeaf(string _name, PropertyBranch * _parent, PropType _type) 
	: name(_name),
	  parent(_parent),
	  value(new PropertyValue(_type, this))
{}

PropertyLeaf::PropertyLeaf(string _name, PropertyBranch * _parent, PropertyValue * _value) 
	: name(_name),
	  parent(_parent),
	  value(_value)
{
	if( value == NULL )
		value = new PropertyValue(this); // create a non_def Value
	else
		value->setParent(this);
}

PropertyLeaf::~PropertyLeaf()
{
	delete value;
}

string PropertyLeaf::getName() const
{
	return name;
}

PropertyBranch * PropertyLeaf::getParent()
{
	return parent;
}

PropertyValue * PropertyLeaf::getValue()
{
	return value;
}

void PropertyLeaf::print(string prepend) const
{
	cout << prepend << name << " = ";
	value->print();
	cout << endl;
}


void PropertyLeaf::printOn(ostream& o) const
{
	o << value->getPath() << " = " << value << endl;
}

ostream & operator<< (ostream & o, PropertyLeaf & _leaf)
{
	_leaf.printOn(o);
	return o;
}

ostream & operator<< (ostream & o, PropertyLeaf * _leaf)
{
	_leaf->printOn(o);
	return o;
}


