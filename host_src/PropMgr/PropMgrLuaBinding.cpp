//
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

using namespace std;

#include "PropMgrLuaBinding.h"


PHCCScriptLua::PHCCScriptLua(lua_State *L)
{
}

PHCCScriptLua::~PHCCScriptLua()
{
}

int PHCCScriptLua::getProperty(lua_State *L)
{  // useage from lua: x:getProperty('/path/to/property')
	// get no of args
	//int args = lua_gettop(L);
	if( ! lua_isstring(L, 1) )
	{
		lua_pushstring(L, "first argument needs to be a string!");
		lua_error(L);
	}
	const char * path = lua_tostring(L, 1);
	PropertyValue * pv = mgr.get(path);
	if( pv == NULL )
	{
		lua_pushnil(L);
		return 1;  // number of returned values
	}
	switch( pv->getType() )
	{
		case PROPTYPE_NOTDEF: lua_pushnil(L); break;
		case PROPTYPE_INT:    lua_pushnumber(L, pv->getInt()); break;
		case PROPTYPE_FLOAT:  lua_pushnumber(L, pv->getFloat()); break;
		case PROPTYPE_STRING: lua_pushstring(L, pv->getString().c_str()); break;
		case PROPTYPE_BOOL:   lua_pushboolean(L, pv->getBool()); break;
	}
	return 1;  // number of returned values
}


int PHCCScriptLua::setProperty(lua_State *L) //creates property if it doesnt exist
{  // usage from lua: x:setProperty('/path/to/property', value_to_set_prop_to)
	if( ! lua_isstring(L, 1) )
	{
		lua_pushstring(L, "first argument needs to be a string!");
		lua_error(L);
	}
	const char * path = lua_tostring(L, 1);
	PropertyValue * pv = mgr.get(path);
	if( pv == NULL )
	{
		pv = mgr.addLeaf(path);
	}
	switch( lua_type(L, 2) ) // check for types we can map to the property tree
	{
		case LUA_TNIL:
		case LUA_TNUMBER:
		case LUA_TBOOLEAN:
		case LUA_TSTRING:  break;
		default:  
			lua_pushstring(L, "second argument needs to be either nil, a number, a string, or a boolean!");
			lua_error(L);
	}

	if( pv->getType() == PROPTYPE_NOTDEF )
	{
		switch( lua_type(L, 2) )
		{
			case LUA_TNIL:     /* do nothing, leave new leaf as PROPTYPE_NOTDEF */ break;
			case LUA_TNUMBER:  pv->set((float)lua_tonumber(L,2)); break;
			case LUA_TBOOLEAN: pv->set(lua_toboolean(L,2)); break;
			case LUA_TSTRING:  pv->set(lua_tostring(L,2)); break;
		}
	}
	else
	{
		if( pv->getType() == PROPTYPE_INT && lua_type(L, 2) == LUA_TNUMBER )
		{
			pv->set((int)lua_tonumber(L,2)); // convert to int
		}
		else if(pv->getType() == PROPTYPE_FLOAT && lua_type(L, 2) == LUA_TNUMBER )
			pv->set((float)lua_tonumber(L,2));
		else if(pv->getType() == PROPTYPE_STRING && lua_type(L, 2) == LUA_TSTRING )
			pv->set(lua_tostring(L,2));
		else if(pv->getType() == PROPTYPE_BOOL && lua_type(L, 2) == LUA_TBOOLEAN )
			pv->set(lua_toboolean(L,2));
		else
		{
		}
	}

	lua_pushnumber(L, 1);
	return 1;  // number of returned values
}


int PHCCScriptLua::registerCallback(lua_State *L)
{ // usage from lua: x:registerCallback('/path/in/prop/tree', lua_function_to_be_called)
	if( ! lua_isstring(L, 1) )
	{
		lua_pushstring(L, "first argument needs to be a string!");
		lua_error(L);
	}
	const char * path = lua_tostring(L, 1);
	PropertyValue * pv = mgr.get(path);
	if( pv == NULL )
	{
		lua_pushnil(L);   // callback to non-existant property   TODO: create empty property instead
		return 1;  // number of returned values
	}
	if( ! lua_isfunction(L, 2) )
	{
		lua_pushstring(L, "second argument needs to be a function!");
		lua_error(L);		
	}
	pv->registerOnChange(new PHCCScriptCallback(L, lua_ref(L,1)));
	lua_pushnumber(L, 1);
	cout << "[lua callback registered sucessfully]" << endl;
	return 1;
}

int PHCCScriptLua::registerTimerOneshot(lua_State *L)
{ // usage from lua: x:registerTimerOneshot(timeout, '/path/in/prop/tree', new_value)
   if( ! lua_isnumber(L, 1) )
   {
		lua_pushstring(L, "first argument needs to be a number!");
		lua_error(L);
   }
   float timeout = lua_tonumber(L, 1);
   int timeout_sec = (int)floorf(timeout);
   int timeout_usec = (int)((timeout-timeout_sec)*1000000);
   if( ! lua_isstring(L, 2) )
   {
	  lua_pushstring(L, "second argument needs to be a string!");
	  lua_error(L);
   }
   const char * path = lua_tostring(L, 2);
   PropertyValue * pv = mgr.get(path);
   if( pv == NULL )
	{
		lua_pushnil(L);   // callback to non-existant property   TODO: create empty property instead
		return 1;  // number of returned values
	}

   switch( lua_type(L, 3) ) // check for types we can map to the property tree
   {
	  case LUA_TNIL:
	  case LUA_TNUMBER:
	  case LUA_TBOOLEAN:
	  case LUA_TSTRING:  break;
	  default:  
		 lua_pushstring(L, "third argument needs to be either nil, a number, a string, or a boolean!");
		 lua_error(L);
   }
   if( pv->getType() == PROPTYPE_NOTDEF )
   {
	  switch( lua_type(L, 3) )
	  {
		 case LUA_TNIL:     /* do nothing, leave new leaf as PROPTYPE_NOTDEF */ 
			break;
		 case LUA_TNUMBER:  
			mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, (float)lua_tonumber(L,3)));
			break;
		 case LUA_TBOOLEAN: 
			mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, lua_toboolean(L,3)));
			break;
		 case LUA_TSTRING:  
			mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, lua_tostring(L,3)));
			break;
	  }
   }
   else
   {
	  if( pv->getType() == PROPTYPE_INT && lua_type(L, 3) == LUA_TNUMBER )
		 mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, (int)lua_tonumber(L, 3)));
	  else if(pv->getType() == PROPTYPE_FLOAT && lua_type(L, 3) == LUA_TNUMBER )
		 mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, (float)lua_tonumber(L, 3)));
	  else if(pv->getType() == PROPTYPE_STRING && lua_type(L, 3) == LUA_TSTRING )
		 mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, lua_tostring(L,3)));
	  else if(pv->getType() == PROPTYPE_BOOL && lua_type(L, 3) == LUA_TBOOLEAN )
		 mgr.addTimerOneshot(new Timer(&mgr, timeout_sec, timeout_usec, path, lua_toboolean(L,3)));
	  else
	  {
	  }
   }
   lua_pushnumber(L, 1);
   return 1;  // number of returned values
}

const char PHCCScriptLua::className[] = "PHCCScriptLua";

#define method(class, name) {#name, &class::name}

Luna<PHCCScriptLua>::RegType PHCCScriptLua::methods[] = {
  method(PHCCScriptLua, getProperty),
  method(PHCCScriptLua, setProperty),
  method(PHCCScriptLua, registerCallback),
  method(PHCCScriptLua, registerTimerOneshot),
  {0,0}
};

//--------------------------------------------------------

lua_State * LuaState;  //global

int startLua()
{
  lua_State *L = lua_open();
  LuaState = L; // global lua state

  luaopen_base(L);
  luaopen_table(L);
  luaopen_io(L);
  luaopen_string(L);
  luaopen_math(L);
  luaopen_debug(L);

  Luna<PHCCScriptLua>::Register(L);
//   lua_dostring(L, "x=PHCCScriptLua();print(tostring(x:getProperty('/test/wholenumber')))");
//   lua_dostring(L, "x=PHCCScriptLua();x:registerCallback('/test/wholenumber', print)");
//   lua_dostring(L, "x=PHCCScriptLua();x:setProperty('/test/wholenumber', -214)");
//   lua_dostring(L, "x=PHCCScriptLua();x:setProperty('/created/by/lua', nil)");

//  regcb("/property/to/listen/for/change", lua_func);

//  lua_dofile(L, "test.lua");

//  lua_setgcthreshold(L, 0);  // collected garbage
//  lua_close(L);
  return 0;
}

