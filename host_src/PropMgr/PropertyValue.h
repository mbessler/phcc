//PropertyValue.h - This stores the actual variable value. Part of a PropertyLeaf
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

#ifndef _PROPERTYVALUE_H
#define _PROPERTYVALUE_H

#include <sstream>
#include <list>

#include "PropertyMgrCommon.h"
#include "OnChangeObj.h"
#include "TimeStamp.h"

class PropertyLeaf;

class PropertyValue
{
	public:
		PropertyValue(PropertyLeaf * _parent=NULL);
		PropertyValue(PropType _type, PropertyLeaf * _parent=NULL);
		PropertyValue(const PropertyValue & _pv);// copy constructor
		~PropertyValue();
		PropertyLeaf * getParent();
		void setParent(PropertyLeaf * _parent);
		std::string getPath();// return full path name of where I am located in the tree, if at all
		// the overloaded set() are only way to change the internal values!!!
		void set(int val);
		void set(float val);
		void set(std::string val);
		void set(bool val);
		bool set(const std::string & s, enum PropType _t);
		enum PropType getType();
		int getInt() const;
		float getFloat() const;
		std::string getString() const;
		bool getBool() const;
		void registerOnChange(OnChangeObj * oco);
		void unregisterOnChange(OnChangeObj * oco);
		void print() const;
		void printOn(std::ostream& o) const;
		friend std::ostream & operator<< (std::ostream & o, const PropertyValue & _pv);
		friend std::ostream & operator<< (std::ostream & o, const PropertyValue * _pv);
	protected:
		enum PropType type;
		int int_value;
		float float_value;
		std::string string_value;
		bool bool_value;
		PropertyLeaf * parent;
// 		typedef std::vector<OnChangeObj *> OnChangeVector;
// 		typedef std::vector<OnChangeObj *>::iterator OnChangeIterator;
		typedef std::list<OnChangeObj *> OnChangeVector;
		typedef std::list<OnChangeObj *>::iterator OnChangeIterator;
		OnChangeVector OnChangeRegistry;
		TimeStamp timestamp;
};

#endif // _PROPERTYVALUE_H
