//
//
// This file is part of X11GC.
//
//  X11GC is a Glass Cockpit Software Suite for X11,
//  which does NOT use OpenGL but relies only on xlib.
//  Copyright (C) 2003-2005 by Manuel Bessler
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

#ifndef _PROPMGRLUABINDING_H
#define _PROPMGRLUABINDING_H

#include <math.h> // for floorf()

extern "C" 
{
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}

#include "PropertyMgr.h"

#include "luna.h"

extern int startLua();  // actual function provided in PropMgrLuaBinding.cpp

extern PropertyMgr mgr;

class PHCCScriptLua
{
	public:
		PHCCScriptLua(lua_State *L);
		~PHCCScriptLua();
		int getProperty(lua_State *L);
		int setProperty(lua_State *L);
		int registerCallback(lua_State *L);
		int registerTimerOneshot(lua_State *L);
		int sendPHCC(lua_State *L);
	public:
		static const char className[];
		static Luna<PHCCScriptLua>::RegType methods[];
	private:

};


//////////////////////////////////////////////////////////////
class PHCCScriptCallback : public OnChangeObj
{
   public:
		PHCCScriptCallback(lua_State *_L, int _luaFunctionRef) : L(_L),luaFunctionRef(_luaFunctionRef) {}
		void onChange(PropertyValue * _pv)
		{
//			cout << "[calling Lua function '" << luaFunctionRef << "' on change of " << _pv << endl;
			lua_getref(L, luaFunctionRef);
			switch( _pv->getType() )
			{
				case PROPTYPE_INT:
					lua_pushnumber(L, _pv->getInt());
					break;
				case PROPTYPE_FLOAT:
					lua_pushnumber(L, _pv->getFloat());
					break;
				case PROPTYPE_STRING:
					lua_pushstring(L, _pv->getString().c_str());
					break;
				case PROPTYPE_BOOL:
					lua_pushboolean(L, _pv->getBool());
					break;
				case PROPTYPE_NOTDEF:
				default:
					lua_pushnil(L);
			}
			lua_call(L,1,0);
			lua_settop(L, 0); // reset the stack
			//		lua_pop(L, 1); corrupts stack
		}
	private:
		lua_State * L;
		int luaFunctionRef;
};





#endif // _PROPMGRLUABINDING_H
