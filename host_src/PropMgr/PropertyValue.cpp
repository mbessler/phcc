//PropertyValue.cpp - This stores the actual variable value. Part of a PropertyLeaf
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

#include "PropertyValue.h"
#include "PropertyLeaf.h"
#include "PropertyBranch.h"

using namespace std;

PropertyValue::PropertyValue(PropertyLeaf * _parent/*=NULL*/) : 
	type(PROPTYPE_NOTDEF),
	int_value(0), 
	float_value(0.0f), 
	string_value(string("[empty]")), 
	bool_value(false),
	parent(_parent),
	timestamp(TimeStamp())
{
	// allows to set the type later, eg. if type and/or value are not known at instantiation time
	// nothing to do at this point
}

PropertyValue::PropertyValue(PropType _type, PropertyLeaf * _parent/*=NULL*/) :  
	type(_type), 
	int_value(0), 
	float_value(0.0f), 
	string_value(string("[empty]")), 
	bool_value(false),
	parent(_parent),
	timestamp(TimeStamp())
{
}

PropertyValue::PropertyValue(const PropertyValue & _pv) : // copy constructor
	type(_pv.type), 
	int_value(_pv.int_value),
	float_value(_pv.float_value), 
	string_value(_pv.string_value), 
	bool_value(_pv.bool_value),
	parent(_pv.parent)
{
	timestamp = _pv.timestamp;
}

PropertyValue::~PropertyValue()
{
//	parent = NULL;
}

PropertyLeaf * PropertyValue::getParent()
{
	return parent;
}

void PropertyValue::setParent(PropertyLeaf * _parent)
{
	parent = _parent;
}

string PropertyValue::getPath()
{ // return full path name of where I am located in the tree, if at all
	if( parent == NULL )
		return string(".");
	string s = parent->getName();  // get PropertyLeaf thats associated with this PropertyValue
	PropertyBranch * curparent = parent->getParent();  // first PropertyBranch, or NULL for toplevel leaves
	while( curparent != NULL )
	{
		if( ! curparent->isRoot() )
			s = curparent->getName() + "/" + s;
		curparent = curparent->getParent();
	}
	return s = "/" + s;
}

// the overloaded set() are only way to change the internal values!!!

void PropertyValue::set(int val)
{
	if( type == PROPTYPE_NOTDEF)
		type = PROPTYPE_INT;
	if( type != PROPTYPE_INT )
	{
		cerr << __PRETTY_FUNCTION__ 
			  << " cannot change type of PropertyValue after initialization" << endl;
		abort();
	}
	int_value = val;
	timestamp.setNow();
	for( OnChangeIterator iter=OnChangeRegistry.begin(); 
		  iter != OnChangeRegistry.end();
		  ++iter )
		(*iter)->onChange(this);
}

void PropertyValue::set(float val)
{
	if( type == PROPTYPE_NOTDEF)
		type = PROPTYPE_FLOAT;
	if( type != PROPTYPE_FLOAT )
	{
		cerr << __PRETTY_FUNCTION__ 
			  << " cannot change type of PropertyValue after initialization" << endl;
		abort();
	}
	float_value = val;
	timestamp.setNow();
	for( OnChangeIterator iter=OnChangeRegistry.begin(); 
		  iter != OnChangeRegistry.end();
		  ++iter )
		(*iter)->onChange(this);
}

void PropertyValue::set(string val)
{
	if( type == PROPTYPE_NOTDEF)
		type = PROPTYPE_STRING;
	if( type != PROPTYPE_STRING )
	{
		cerr << __PRETTY_FUNCTION__ 
			  << " cannot change type of PropertyValue after initialization" << endl;
		abort();
	}
	string_value = val;
	timestamp.setNow();
	for( OnChangeIterator iter=OnChangeRegistry.begin(); 
		  iter != OnChangeRegistry.end();
		  ++iter )
		(*iter)->onChange(this);
}

void PropertyValue::set(bool val)
{
	if( type == PROPTYPE_NOTDEF)
		type = PROPTYPE_BOOL;
	if( type != PROPTYPE_BOOL )
	{
		cerr << __PRETTY_FUNCTION__ 
			  << " cannot change type of PropertyValue after initialization" << endl;
		abort();
	}
	bool_value = val;
	timestamp.setNow();
	for( OnChangeIterator iter=OnChangeRegistry.begin(); 
		  iter != OnChangeRegistry.end();
		  ++iter )
		(*iter)->onChange(this);
}

bool PropertyValue::set(const string & s, enum PropType _t)
{
	int ival;
	float fval;
	bool bval;
	istringstream iss(s);
	switch( _t )
	{
		case PROPTYPE_INT:
			if( iss >> ival ) set(ival);
			else return false;
			break;
		case PROPTYPE_FLOAT:
			if( iss >> fval ) set(fval);
			else return false;
			break;
		case PROPTYPE_STRING:
			set(s);
			break;
		case PROPTYPE_BOOL:
			if( iss >> bval ) set(bval);
			else return false;
			break;
		case PROPTYPE_NOTDEF:
		default:
			return false;
	}
	return true;
}

enum PropType PropertyValue::getType()
{
	return type;
}

int PropertyValue::getInt() const
{
	return int_value;
}

float PropertyValue::getFloat() const
{
	return float_value;
}

string PropertyValue::getString() const
{
	return string_value;
}

bool PropertyValue::getBool() const
{
	return bool_value;
}

void PropertyValue::registerOnChange(OnChangeObj * oco)
{
	OnChangeRegistry.push_back(oco);
}

void PropertyValue::unregisterOnChange(OnChangeObj * oco)
{
   bool repeat = true;
   do
   {
	  for( OnChangeIterator iter=OnChangeRegistry.begin(); 
		   iter != OnChangeRegistry.end();
		   ++iter )
		 if( (*iter) == oco )
		 {
			OnChangeRegistry.erase(iter);
			repeat = true;
			break;
		 }
		 else
			repeat = false;
	  if( OnChangeRegistry.begin() == OnChangeRegistry.end() )
		 repeat = false;
   } while( repeat == true );
}

void PropertyValue::print() const
{
	printOn(cout);
}

void PropertyValue::printOn(ostream& o) const
{
	switch(type)
	{
		case PROPTYPE_NOTDEF: o << "[not defined](notdef) ["    << timestamp << "]"; break;
		case PROPTYPE_INT:    o << int_value    << "(int) ["    << timestamp << "]"; break;
		case PROPTYPE_FLOAT:  o << float_value  << "(float) ["  << timestamp << "]"; break;
		case PROPTYPE_STRING: o << "\"" << string_value << "\"(string) [" << timestamp << "]"; break;
		case PROPTYPE_BOOL:   o << bool_value   << "(bool) ["   << timestamp << "]"; break;
	}			
}

ostream & operator<< (ostream & o, const PropertyValue & _pv)
{
	_pv.printOn(o);
	return o;
}

ostream & operator<< (ostream & o, const PropertyValue * _pv)
{
	_pv->printOn(o);
	return o;
}

