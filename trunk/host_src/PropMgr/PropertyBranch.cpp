//PropertyBranch.cpp - The (sub-)branches of a Property Tree
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

#include "PropertyBranch.h"
#include "PropertyLeaf.h"
#include "PropertyValue.h"

using namespace std;

PropertyBranch::PropertyBranch(string _name, PropertyBranch * _parent, bool _isroot/*=false*/) 
	: name(_name), 
	  parent(_parent),
	  isroot(_isroot)
{}

PropertyBranch::~PropertyBranch()
{
 	for( leaves_iter iter=leaves.begin(); iter != leaves.end(); ++iter )
 		delete (*iter).second;
	for( branches_iter iter=branches.begin(); iter != branches.end(); ++iter )
		delete (*iter).second;
}

string PropertyBranch::getName() const
{
	return name;
}

bool PropertyBranch::isRoot() const
{
	return isroot;
}

PropertyBranch * PropertyBranch::getParent()
{
	if( isroot )
		return NULL;
	return parent;
}

PropertyBranch * PropertyBranch::findBranch(string _name)
{
	branches_iter iter =	branches.find(_name);
	if( iter != branches.end() )
		return (*iter).second;
	else
		return NULL;
}

PropertyLeaf * PropertyBranch::findLeaf(string _name)
{
	leaves_iter iter = leaves.find(_name);
	if( iter != leaves.end() )
		return (*iter).second;
	else
		return NULL;
}
		
void PropertyBranch::add(PropertyBranch * _branch)
{
//TODO: check if branch w/ same name exists
	branches_pair tmp;
	tmp.first = _branch->getName();
	tmp.second = _branch;
	branches.insert(tmp);
}

void PropertyBranch::add(PropertyLeaf * _leaf)
{
//TODO: check if leaf w/ same name exists
	leaves_pair tmp;
	tmp.first = _leaf->getName();
	tmp.second = _leaf;
	leaves.insert(tmp);
}

void PropertyBranch::print(string prepend)
{
	for( leaves_iter iter=leaves.begin(); iter != leaves.end(); ++iter )
		if( isroot )
			(*iter).second->print(prepend);
		else
			(*iter).second->print(prepend+name+"/");
	for( branches_iter iter=branches.begin(); iter != branches.end(); ++iter )
		if( isroot )
			(*iter).second->print(prepend);
		else
			(*iter).second->print(prepend+name+"/");
}


void PropertyBranch::printOn(ostream& o)
{
	for( branches_iter iter=branches.begin(); iter != branches.end(); ++iter )
		o << (*iter).second;
	for( leaves_iter iter=leaves.begin(); iter != leaves.end(); ++iter )
		o << (*iter).second;
}

ostream & operator<< (ostream & o, PropertyBranch & _branch)
{
	_branch.printOn(o);
	return o;
}

ostream & operator<< (ostream & o, PropertyBranch * _branch)
{
	_branch->printOn(o);
	return o;
}

