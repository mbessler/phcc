//PropertyBranch.h - The (sub-)branches of a Property Tree
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

#ifndef _PROPERTYBRANCH_H
#define _PROPERTYBRANCH_H

#include "PropertyMgrCommon.h"

class PropertyLeaf;
class PropertyValue;

class PropertyBranch
{
	public:
		PropertyBranch(std::string _name, PropertyBranch * _parent, bool _isroot=false);
		~PropertyBranch();
		std::string getName() const;
		bool isRoot() const;
		PropertyBranch * getParent();
		PropertyBranch * findBranch(std::string _name);
		PropertyLeaf * findLeaf(std::string _name);
		void add(PropertyBranch * _branch);
		void add(PropertyLeaf * _leaf);
		void print(std::string prepend);
		void printOn(std::ostream& o);
		friend std::ostream & operator<< (std::ostream & o, PropertyBranch & _branch);
		friend std::ostream & operator<< (std::ostream & o, PropertyBranch * _branch);
	protected:
		std::string name;
		PropertyBranch * parent;
		bool isroot;

		typedef std::map<std::string, PropertyLeaf *> leaves_map;
		typedef std::pair<std::string, PropertyLeaf *> leaves_pair;
		typedef leaves_map::iterator leaves_iter;
		leaves_map leaves;

		typedef std::map<std::string, PropertyBranch *> branches_map;
		typedef std::pair<std::string, PropertyBranch *> branches_pair;
		typedef branches_map::iterator branches_iter;
		branches_map branches;
};


#endif // _PROPERTYBRANCH_H
