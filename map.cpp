/*
** map.cc
** Login : <mouret@asuncion.lip6.fr>
** Started on  Mon Jan 14 16:39:08 2008 Jean-Baptiste MOURET
** $Id$
** 
** Copyright (C) 2008 Jean-Baptiste MOURET
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
** 
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
** 
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <iostream>
#include <sstream>
#include <list>
#include "map.hpp"
#include "robot.hpp"

namespace fastsim_multi
{
  void Map::_read_file(const std::string& fname)
  {
    std::string str;
    std::ifstream ifs(fname.c_str());
    if (!ifs.good())
      throw Exception(std::string("cannot open map :") + fname);
    
    ifs >>str;
    if (str != "P4")
      throw Exception("wrong file type for map");
    char line[256];
    ifs.getline(line,256);
    while ((line[0]=='\0')||(line[0] == '#')) {
      ifs.getline(line,256);
    }
    std::istringstream istr(line);
    istr>> _w>>_h;
    if ((_w<=0)||(_h<=0)) {
      std::cerr<<"ERROR: the size of your map is not valid. There may be a problem with your file."<<std::endl;
      std::cerr<<" w="<<_w<<" h="<<_h<<std::endl;
      std::cerr<<" file="<<fname<<std::endl;
      exit(1);
    }
    _data.resize(_w * _h);
    if (_w % 8 != 0)
      throw Exception("wrong map size");
    int k = _w * _h / 8;
    char buffer[k];

    ifs.read((char*)buffer, k);
    for (int i = 0; i < k; ++i)
      for (int j = 0; j < 8; ++j)
	_data[i * 8 + j] = _get_bit(buffer[i ], j) ? obstacle : free;
  }
  
  // we use the "triangle method"
  // concept : 
  // * area = base * height / 2
  // * area = cross product
  // -> if height < radius, intersection
  bool Map::_check_inter_ray_circle(float x1, float y1,
				    float x2, float y2,
				    float xr, float yr, float radius) const
  {
    // check if the object is on the right side of the camera
    float dot = (x2 - x1) * (xr - x1) + (y2 - y1) * (yr - y1);
    if (dot < 0)
      return false;
    float area2 = fabs((x2 - x1) * (yr - y1) -  (y2 - y1) * (xr - x1));
    float dx = x2 - x1;
    float dy = y2 - y1;
    float lab = sqrt(dx * dx + dy * dy);
    float h = area2 / lab;
    if (h < radius)
      return true;
    return false;
  }
  
  int Map::check_inter_is(float x1, float y1,
			  float x2, float y2) const 
  {
    // list intersections with rays
    std::vector<ill_sw_t> res;
    for (size_t i = 0; i < _illuminated_switches.size(); ++i)
      {
	ill_sw_t isv = _illuminated_switches[i];
	float xtmp, ytmp;
	if (isv->get_on()
	    && _check_inter_ray_circle(x1, y1, x2, y2,
				      isv->get_x(), isv->get_y(),
				      isv->get_radius())
	    && !check_inter_real(x1, y1, isv->get_x(), isv->get_y(), xtmp, ytmp))
	  res.push_back(isv);	    
      }
    if (res.empty())
      return -1;
    // return the closest
    std::sort(res.begin(), res.end(), ClosestSwitch_f(x1, y1));
    return res[0]->get_color();
  }
  

  Map::Return_check_t Map::check_inter_is_dist(float x1, float y1,
        float x2, float y2) const 
  {
    // list intersections with rays
    Return_check_t ret;
    ret.pixel = -1;
    ret.dist = -1;

    ClosestObject_f closest(x1, y1);

    // We look for an intersection with any switch
    bool ill_sw_found = false;
    ill_sw_t res_sw;
    for (size_t i = 0; i < _illuminated_switches.size(); ++i)
    {
      ill_sw_t isv = _illuminated_switches[i];

      // If we didn't find any switch yet or if this 
      // switch is closer than the found one
      if(!ill_sw_found || closest(isv->get_x(), isv->get_y(), res_sw->get_x(), res_sw->get_y()))
      {
        float xtmp, ytmp;
        if (isv->get_on()
            && _check_inter_ray_circle(x1, y1, x2, y2,
                    isv->get_x(), isv->get_y(),
                    isv->get_radius())
            && !check_inter_real(x1, y1, isv->get_x(), isv->get_y(), xtmp, ytmp))
        {
          res_sw = isv;
          ill_sw_found = true;
        }
      }
    }
    // std::cout << "Illuminated switch found : " << ill_sw_found << std::endl;

    // We look for an intersection with any robot
    bool robot_found = false;
    Robot* res_robot = NULL;
    for (size_t i = 0; i < _robots.size(); ++i)
    {
      Robot* robot = _robots[i];

      if(robot->is_on())
      {
        // Either :
        //  - We already found a robot (and consequently it is closer than the illuminated switch) and we
        //    want to see if this one is closer
        //  - Or we didn't find a robot and if we previously found an illuminated switch we want to see if
        //    this robot is closer
        if((robot_found && closest(robot->get_pos().x(), robot->get_pos().y(), res_robot->get_pos().x(), res_robot->get_pos().y()))
          || (!robot_found && (!ill_sw_found || closest(robot->get_pos().x(), robot->get_pos().y(), res_sw->get_x(), res_sw->get_y()))))
        {
          if(_check_inter_ray_circle(x1, y1, x2, y2,
                      robot->get_pos().x(), robot->get_pos().y(),
                      robot->get_radius()))
          {
            // To avoid collision problems, the ray is traced from (x1, y1) to the
            // outer circle of the robot. It should also make the camera more precise

            // The angle between the two robots
            float angle = normalize_angle(atan2(robot->get_pos().y() - y1, robot->get_pos().x() -x1));

            // Intersection point between the radius circle and the ray
            float xinter, yinter;
            xinter = robot->get_pos().x() - (cos(angle)*(robot->get_radius() + 3));
            yinter = robot->get_pos().y() - (sin(angle)*(robot->get_radius() + 3));

            float xtmp, ytmp;

            if (!check_inter_real(x1, y1, xinter, yinter, xtmp, ytmp))
            {
              res_robot = robot;
              robot_found = true;
            }
          }
        }
      }
    }
    // std::cout << "Robot found : " << robot_found << std::endl;

    if(robot_found)
    {
      ret.pixel = res_robot->color();

      float x_robot = res_robot->get_pos().x();
      float y_robot = res_robot->get_pos().y();
      float x_dist = x_robot - x1;
      float y_dist = y_robot - y1;
      float dist = sqrtf(x_dist * x_dist + y_dist * y_dist);
      ret.dist = dist - res_robot->get_radius();
    }
    else
    {
      if(ill_sw_found)
      {
        ret.pixel = res_sw->get_color();

        float x_sw = res_sw->get_x();
        float y_sw = res_sw->get_y();
        float x_dist = x_sw - x1;
        float y_dist = y_sw - y1;
        float dist = sqrtf(x_dist * x_dist + y_dist * y_dist);
        ret.dist = dist - res_sw->get_radius();
      }
    }

    return ret;
  }
  
  
  bool Map :: _try_pixel(int x, int y) const
  {
    if (x >= 0 && y >= 0 
	&& x < get_pixel_w() 
	&& y < get_pixel_h() 
	&& get_pixel(x, y) == free)
      return false;
    else
      return true;
  }


  
  // see
  // http://lifc.univ-fcomte.fr/~dedu/projects/bresenham/index.html
  // In PIXEL coordinates
  bool Map :: check_inter_pixel(int y1, int x1,
				int y2, int x2,
				int& y_res, int& x_res) const
  {
    int i;               // loop counter
    int ystep, xstep;    // the step on y and x axis
    int error;           // the error accumulated during the increment
    int errorprev;       // *vision the previous value of the error variable
    int y = y1, x = x1;  // the line points
    int ddy, ddx;        // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    bool inter = _try_pixel(y1, x1);
    if (dy < 0) { ystep = -1; dy = -dy; } else ystep = 1;
    if (dx < 0) { xstep = -1; dx = -dx; } else xstep = 1;
    ddy = dy * 2;
    ddx = dx * 2;
    if (ddx >= ddy) // first octant (0 <= slope <= 1)
    {  
    	errorprev = error = dx;
    	for (i = 0 ; i < dx ; i++)
  	  {  // do not use the first point (already done)
  	    x += xstep;
  	    error += ddy;
  	    if (error > ddx)
        {
  		    y += ystep;
  		    error -= ddx;
  		    if (error + errorprev < ddx)  // bottom square also
  		      inter = inter || _try_pixel(y - ystep, x);
  		    else if (error + errorprev > ddx)  // left square also
  		      inter = inter || _try_pixel(y, x - xstep);
  		    else // corner: bottom and left squares also
  		    {
  		      inter = inter || _try_pixel(y - ystep, x);
  		      inter = inter || _try_pixel(y, x - xstep);
  		    }
        }
  	    inter = inter || _try_pixel(y, x);
  	    errorprev = error;
  	    if (inter)
        {
  		    x_res = x;
  		    y_res = y;
  		    return true;
	      }
  	  }
    }
    else
    {  // the same as above
    	errorprev = error = dy;
    	for (i = 0 ; i < dy ; i++)
  	  {
  	    y += ystep;
  	    error += ddx;
  	    if (error > ddy){
  	      x += xstep;
  	      error -= ddy;
  	      if (error + errorprev < ddy)
        		inter = inter || _try_pixel(y, x - xstep);
  	      else if (error + errorprev > ddy)
        		inter = inter || _try_pixel(y - ystep, x);
  	      else
    		  {
      		  inter = inter || _try_pixel(y, x - xstep);
      		  inter = inter || _try_pixel(y -  ystep, x);
    		  }
        }
  	    inter = inter || _try_pixel(y, x);
  	    errorprev = error;
  	    if (inter)
	      {
      		x_res = x;
      		y_res = y;
      		return true;
	      }
    	}
    }

    return false;
  }

    // Draws a rectangle with (x,y) the upper left point and (lx,ly) the size
  void Map:: draw_rect(int x, int y, int lx, int ly) {
    int i,j;

    for (i=0;i<lx;i++)
      for (j=0;j<ly;j++) {
	if ((x+i) >= 0 && (y+j) >= 0 
	    && (x+i) < get_pixel_w() 
	    && (y+j) < get_pixel_h())
	  set_pixel(x+i,y+j,obstacle);
      }
  }
  
  void Map :: clean_robot_obstacle(Robot* robot)
  {
    float x = robot->get_pos().x();
    float y = robot->get_pos().y();
    float radius = robot->get_radius();
    float bbx = robot->get_bb().x;
    float bby = robot->get_bb().y;
    float bbw = robot->get_bb().w;
    float bbh = robot->get_bb().h;

    int _rp = real_to_pixel(radius);
    int _r = _rp * _rp;
    int _x = real_to_pixel(x);
    int _y = real_to_pixel(y);
    int _bbx = real_to_pixel(bbx);
    int _bby = real_to_pixel(bby);
    int _bbw = real_to_pixel(bbx + bbw);
    int _bbh = real_to_pixel(bby + bbh);
    
    typedef std::pair<int, int> p_t;
    std::list<p_t > coll_points;
    for (int i = _bbx; i < _bbw; ++i)
      for (int j = _bby; j < _bbh; ++j)
        if (get_pixel(i, j) == Map::obstacle)
        {
          float d1 = (i - _x);
          float d2 = (j - _y);
          if (d1 * d1 + d2 * d2 <= _r)
            set_pixel(i, j, Map::free);
        }
  }
   
  void Map :: add_robot_obstacle(Robot* robot)
  {
    float x = robot->get_pos().x();
    float y = robot->get_pos().y();
    float radius = robot->get_radius();
    float bbx = robot->get_bb().x;
    float bby = robot->get_bb().y;
    float bbw = robot->get_bb().w;
    float bbh = robot->get_bb().h;

    int _rp = real_to_pixel(radius);
    int _r = _rp * _rp;
    int _x = real_to_pixel(x);
    int _y = real_to_pixel(y);
    int _bbx = real_to_pixel(bbx);
    int _bby = real_to_pixel(bby);
    int _bbw = real_to_pixel(bbx + bbw);
    int _bbh = real_to_pixel(bby + bbh);
    
    typedef std::pair<int, int> p_t;
    std::list<p_t > coll_points;
    for (int i = _bbx; i < _bbw; ++i)
      for (int j = _bby; j < _bbh; ++j)
        if (get_pixel(i, j) == Map::free)
        {
          float d1 = (i - _x);
          float d2 = (j - _y);
          if (d1 * d1 + d2 * d2 <= _r)
            set_pixel(i, j, Map::obstacle);
        }
  }
}
