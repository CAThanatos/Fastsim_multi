/*
** sdl.cc
** Login : <mouret@mexico>
** Started on  Sat Jan 12 20:40:12 2008 Jeanbaptiste MOURET
** $Id$
** 
** Copyright (C) 2008 Jeanbaptiste MOURET
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
#include "display.hpp"
#include <bitset>

#ifdef USE_SDL

namespace fastsim_multi
{

  void Display :: _events()
  {
    SDL_Event event;
    while (SDL_PollEvent(&event))
      {
	switch (event.type) 
	  {
	  case SDL_QUIT:
	    _quit();
	  case SDL_KEYDOWN:
	    if (event.key.keysym.sym == SDLK_ESCAPE)
	      _quit();
	    break;
	  }
      }
  }

  void Display :: _blit_map()
  {
    for (unsigned i = 0; i < _map->get_pixel_w(); ++i)
      for (unsigned j = 0; j < _map->get_pixel_h(); ++j)
      	if (_map->get_pixel(i, j) == Map::obstacle)
        {
      	  _put_pixel(_map_bmp, i, j, 0, 0, 0);
          _put_pixel(_map_bmp_no_sensors, i, j, 0, 0, 0);
        }
      	else
        {
      	  _put_pixel(_map_bmp, i, j, 255, 255, 255);
          _put_pixel(_map_bmp_no_sensors, i, j, 255, 255, 255);
        }

    if(_display_sensors)
    {
      for (unsigned i = _map->get_pixel_w(); i < _map->get_pixel_w()*2; ++i)
        for (unsigned j = 0; j < _map->get_pixel_h(); ++j)
          _put_pixel(_map_bmp, i, j, 255, 255, 255);
    }

    SDL_BlitSurface(_map_bmp, 0, _screen, 0);

    if(_no_video)
      _screen = SDL_ConvertSurface(_map_bmp, _map_bmp->format, SDL_SWSURFACE);
    else
      SDL_UpdateRect(_screen, 0, 0, _w, _h);
  }

  Display :: Display(const boost::shared_ptr<Map>& m, const std::vector<Robot*>& robots, bool display_sensors, bool no_video) : 
    _map(m), _robots(robots), _selected(0), _display_sensors(display_sensors), _no_video(no_video)
  {
    _w = _map->get_pixel_w();
    _h = _map->get_pixel_h();

    if(!_no_video)
    {
      if (SDL_Init(SDL_INIT_VIDEO) == -1)
        throw Exception(SDL_GetError());

      if(_display_sensors)
        _screen = SDL_SetVideoMode(_w*2, _h, 32, SDL_SWSURFACE);
      else
        _screen = SDL_SetVideoMode(_w, _h, 32, SDL_SWSURFACE);

      if (!_screen)
        throw Exception(SDL_GetError());
    }

    Uint32 rmask, gmask, bmask, amask;
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
    rmask = 0xff000000;
    gmask = 0x00ff0000;
    bmask = 0x0000ff00;
    amask = 0x000000ff;
#else
    rmask = 0x000000ff;
    gmask = 0x0000ff00;
    bmask = 0x00ff0000;
    amask = 0xff000000;
#endif

    if(_no_video)
    {
      if(_display_sensors)
        _screen = SDL_CreateRGBSurface(SDL_SWSURFACE, _w*2, _h, 32, rmask, gmask, bmask, amask);
      else
        _screen = SDL_CreateRGBSurface(SDL_SWSURFACE, _w, _h, 32, rmask, gmask, bmask, amask);
    }

    if (!_screen)
      throw Exception(SDL_GetError());

    if(_display_sensors)
      _map_bmp = SDL_CreateRGBSurface(SDL_SWSURFACE, _w*2, _h, 32, 
            rmask, gmask, bmask, amask);
    else
      _map_bmp = SDL_CreateRGBSurface(SDL_SWSURFACE, _w, _h, 32, 
				    rmask, gmask, bmask, amask);

      _map_bmp_no_sensors = SDL_CreateRGBSurface(SDL_SWSURFACE, _w, _h, 32, 
            rmask, gmask, bmask, amask);    

    _blit_map();
  }
  
  void Display :: init_robots()
  {
    _trajectory_log.resize(_robots.size());

    for(size_t i = 0; i < _robots.size(); ++i)
    {
      // We need to fill the _prev_bb vector
      SDL_Rect prev_bb;
      _prev_bb.push_back(prev_bb);
      _bb_to_sdl(_robots[i]->get_bb(), &_prev_bb[i]);
    }
  }

  void Display :: _line(SDL_Surface* surf,
			int x0, int y0, int x1, int y1, 
			Uint32 color)
  {
/*  // Doesn't seem to bother it if x0, y0, x1 or x1 are out
    // of bounds (and this code makes the lines buggy) 
    x0 = std::max(x0, 0); x0 = std::min(x0, surf->w - 1);
    x1 = std::max(x1, 0); x1 = std::min(x1, surf->w - 1);
    y0 = std::max(y0, 0); y0 = std::min(y0, surf->h - 1);
    y1 = std::max(y1, 0); y1 = std::min(y1, surf->h - 1);*/

    int dy = y1 - y0;
    int dx = x1 - x0;
    int stepx, stepy;

    if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
    if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
    dy <<= 1; // dy is now 2*dy
    dx <<= 1; // dx is now 2*dx

    _put_pixel(surf, color, x0, y0);
    if (dx > dy) 
      {
	int fraction = dy - (dx >> 1); // same as 2*dy - dx
	while (x0 != x1)
	  {
	    if (fraction >= 0)
	      {
		y0 += stepy;
		fraction -= dx; // -= 2*dx
	      }
	    x0 += stepx;
	    fraction += dy; //  -= 2*dy
	    _put_pixel(surf, color, x0, y0);
	  }
      } 
    else 
      {
	int fraction = dx - (dy >> 1);
	while (y0 != y1)
	  {
	    if (fraction >= 0)
	      {
		x0 += stepx;
		fraction -= dy;
	      }
	    y0 += stepy;
	    fraction += dx;
	    _put_pixel(surf, color, x0, y0);
	  }
      }
  }

  void Display :: _try_pixel(bool& res,
			     SDL_Surface* surf,
			     Uint32 color, int x, int y)
  {

    if (x >= 0 && y >= 0 
	&& x < _map->get_pixel_w() 
	&& y < _map->get_pixel_h() 
	&& _map->get_pixel(x, y) == Map::free)
      {
	_put_pixel(surf, color, x, y);
	res = false;
      }
    else
      res = true;
  }

  void Display :: _circle_points(SDL_Surface* surf,
				 int cx, int cy, int x, int y, Uint32 color)
  {

    if (x == 0) {
      _put_pixel(surf, color, cx, cy + y);
      _put_pixel(surf, color, cx, cy - y);
      _put_pixel(surf, color, cx + y, cy);
      _put_pixel(surf, color, cx - y, cy);
    }
    else if (x == y) {
      _put_pixel(surf, color, cx + x, cy + y);
      _put_pixel(surf, color, cx - x, cy + y);
      _put_pixel(surf, color, cx + x, cy - y);
      _put_pixel(surf, color, cx - x, cy - y);
    } else if (x < y) {
      _put_pixel(surf, color, cx + x, cy + y);
      _put_pixel(surf, color, cx - x, cy + y);
      _put_pixel(surf, color, cx + x, cy - y);
      _put_pixel(surf, color, cx - x, cy - y);
      _put_pixel(surf, color, cx + y, cy + x);
      _put_pixel(surf, color, cx - y, cy + x);
      _put_pixel(surf, color, cx + y, cy - x);
      _put_pixel(surf, color, cx - y, cy - x);
    }
  }

  void Display :: _circle(SDL_Surface *surf,
			  int x_center, int y_center, int radius,
			  Uint32 color)
  {    
    int x = 0;
    int y = radius;
    int p = (5 - radius * 4) / 4;
    
    _circle_points(surf, x_center, y_center, x, y, color);
    while (x < y) 
      {
	x++;
	if (p < 0) 
	  p += 2 * x + 1;
	else
	  {
	    y--;
	    p += 2 * (x - y) + 1;
	  }
	_circle_points(surf, x_center, y_center, x, y, color);
      }
  }
  /// see http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
  // transform setPixel(x0 + x, y0 + y); setPixel(x0 - x, y0 + y);
  // to  _line(x0 - x,, y0+y, x0 + x, y0 + y)
  void Display :: _disc(SDL_Surface *surf,
			int x_center, int y_center, int radius,
			Uint32 color)
  {
    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;
    int x0 = x_center;
    int y0 = y_center;

    _line(surf, x0, y0 - radius, x0, y0 + radius, color);
    _line(surf, x0 - radius, y0, x0 + radius, y0, color);    
    while(x < y)
      {
	// ddF_x == 2 * x + 1;
	// ddF_y == -2 * y;
	// f == x*x + y*y - radius*radius + 2*x - y + 1;
	if(f >= 0) 
	  {
	    y--;
	    ddF_y += 2;
	    f += ddF_y;
	  }
	x++;
	ddF_x += 2;
	f += ddF_x;   
	_line(surf, x0 - x, y0 + y, x0 + x, y0 + y, color);
	_line(surf, x0 - x, y0 - y, x0 + x, y0 - y, color);
	_line(surf, x0 - y, y0 + x, x0 + y, y0 + x, color);
	_line(surf, x0 - y, y0 - x, x0 + y, y0 - x, color);
      }
  }



  void Display :: _disp_bb()
  {
    for(std::vector<fastsim_multi::Robot*>::const_iterator it = _robots.begin(); it != _robots.end(); ++it) 
    {
      unsigned x = _map->real_to_pixel((*it)->get_bb().x);
      unsigned y = _map->real_to_pixel((*it)->get_bb().y);
      unsigned w = _map->real_to_pixel((*it)->get_bb().w);
      unsigned h = _map->real_to_pixel((*it)->get_bb().h);
      
      assert(x >= 0);
      assert(y >= 0);
      assert(x + w < _screen->w);
      assert(y + h < _screen->h);
      _line(_screen, x, y, x + w, y, 0);
      _line(_screen, x + w, y, x + w, y + h, 0);
      _line(_screen, x + w, y + h, x, y + h, 0);
      _line(_screen, x, y + h, x, y, 0);    
    }
  }

  void Display :: _disp_goals()
  {
    for (size_t i = 0; i < _map->get_goals().size(); ++i)
      {
	const Goal& goal = _map->get_goals()[i];
	unsigned x = _map->real_to_pixel(goal.get_x());
	unsigned y = _map->real_to_pixel(goal.get_y());
	unsigned diam = _map->real_to_pixel(goal.get_diam());
	Uint8 r = 0, g = 0, b = 0;
	switch (goal.get_color())
	  {
	  case 0:
	    r = 255;
	    break;
	  case 1:
	    g = 255;
	    break;
	  case 2:
	    b = 255;
	    break;
	  default:
	    assert(0);
	  }
	_circle(_screen, x, y, diam, r, g, b);
      }    
  }

  void Display :: _disp_radars()
  {
    for(std::vector<fastsim_multi::Robot*>::const_iterator it = _robots.begin(); it != _robots.end(); ++it) 
    {
      unsigned r = _map->real_to_pixel((*it)->get_radius())  / 2;
      unsigned x = _map->real_to_pixel((*it)->get_pos().x());
      unsigned y = _map->real_to_pixel((*it)->get_pos().y());

      for (size_t i = 0; i < (*it)->get_radars().size(); ++i)
      {
        const Radar& radar = (*it)->get_radars()[i];
        if (radar.get_activated_slice() != -1)
        {
          float a1 = (*it)->get_pos().theta() + radar.get_inc() * radar.get_activated_slice();
          float a2 = (*it)->get_pos().theta() + radar.get_inc() * (radar.get_activated_slice() + 1);
          _line(_screen,
          cos(a1) * r + x, sin(a1) * r + y,
          cos(a2) * r + x, sin(a2) * r + y,
          0x0000FF);
          const Goal& g = _map->get_goals()[radar.get_color()];
          unsigned gx = _map->real_to_pixel(g.get_x());
          unsigned gy = _map->real_to_pixel(g.get_y());     
          _line(_screen, x, y, gx, gy, 0x0000FF);
        }
      }
    }
  }

  void Display :: _disp_bumpers()
  {
    for(std::vector<fastsim_multi::Robot*>::const_iterator it = _robots.begin(); it != _robots.end(); ++it) 
    {
      // convert to pixel
      unsigned x = _map->real_to_pixel((*it)->get_pos().x());
      unsigned y = _map->real_to_pixel((*it)->get_pos().y());
      unsigned r = _map->real_to_pixel((*it)->get_radius());
      float theta = (*it)->get_pos().theta();
      Uint32 cb_left = SDL_MapRGB(_screen->format, (*it)->get_left_bumper() ? 255 : 0, 0, 0);
      Uint32 cb_right = SDL_MapRGB(_screen->format, (*it)->get_right_bumper() ? 255 : 0, 0, 0);
      _line(_screen,
      (int) (r * cosf(theta + M_PI / 2.0f) + x),
      (int) (r * sinf(theta + M_PI / 2.0f) + y),
      (int) (r * cosf(theta) + x),
      (int) (r * sinf(theta) + y),
      cb_left);
      _line(_screen,
      (int) (r * cosf(theta - M_PI / 2.0f) + x),
      (int) (r * sinf(theta - M_PI / 2.0f) + y),
      (int) (r * cosf(theta) + x),
      (int) (r * sinf(theta) + y),
      cb_right);
    }
  }

  Uint32 Display :: _color_from_id(SDL_Surface* surf, int x)
  {
    std::bitset<4> bs = x;	
    Uint8 k = 1 - bs[4];
    Uint8 r = bs[0] * 128 + 127 * k;
    Uint8 g = bs[1] * 128 + 127 * k;
    Uint8 b = bs[2] * 128 + 127 * k;
    
    return SDL_MapRGB(surf->format, r, g, b);
  }
  
  void Display :: _disp_switches()
  {
    for (size_t i = 0; i < _map->get_illuminated_switches().size(); ++i)
    {
    	const IlluminatedSwitch& sw = *_map->get_illuminated_switches()[i];
    	unsigned x = _map->real_to_pixel(sw.get_x());
    	unsigned y = _map->real_to_pixel(sw.get_y());
    	unsigned rad = _map->real_to_pixel(sw.get_radius());
    	Uint32 color = _color_from_id(_screen, sw.get_color());

    	_circle(_screen, x, y, rad, color);
    	if (sw.get_on())
    	  _disc(_screen, x, y, rad, color);
    }
  }

  void Display :: _disp_lasers()
  {
    for(int j = 0; j < _robots.size(); ++j)
    {
      if(_robots[j]->get_lasers().size() > 0)
      {
        const int px = _w;
        const int py = 40 * (_h/800.0f);
        const int pw = _w;
        const int ph = _h - py;

        SDL_Rect rect;
        rect.x = px;
        rect.y = py;
        rect.w = pw;
        rect.h = ph;

        if(_display_sensors && (j == _selected))
        {
          // Display rectangle
          SDL_FillRect(_screen, &rect, 0xffffffff);
          _line(_screen, px, py, px + pw, py, 0x000000);
          _line(_screen, px, py + ph, px + pw, py + ph, 0x000000);
        }

        // convert to pixel
        unsigned x = px + _w/2;
        unsigned y = py + (ph/2);
        const int r_real = 40 * (_h/800.0f);
        unsigned r = _map->real_to_pixel(r_real);

        // draw the robot
        if(_display_sensors && (j == _selected))
        {
          unsigned int col = _robots[j]->display_color();
          _disc(_screen, x, y, r, _color_from_id(_screen,col));
          _circle(_screen, x, y, r * 5, 0, 0, 255);
        }
                      
        for (size_t i = 0; i < _robots[j]->get_lasers().size(); ++i)
        {
          unsigned x_laser = _map->real_to_pixel(_robots[j]->get_pos().x() 
                         + _robots[j]->get_lasers()[i].get_gap_dist() 
                         * cosf(_robots[j]->get_pos().theta() 
                          + _robots[j]->get_lasers()[i].get_gap_angle()));
          unsigned y_laser = _map->real_to_pixel(_robots[j]->get_pos().y() 
                         + _robots[j]->get_lasers()[i].get_gap_dist() 
                         * sinf(_robots[j]->get_pos().theta()
                          + _robots[j]->get_lasers()[i].get_gap_angle()));

          // _robots[j]->get_lasers()[i].set_xy_pixel(_robots[j]->get_pos(), _map);
          _line(_screen, x_laser, y_laser,
                _robots[j]->get_lasers()[i].get_x_pixel(),
                _robots[j]->get_lasers()[i].get_y_pixel(),
//                0xFF00000);
                0xFF00000);
          
            
          if(_display_sensors && (j == _selected))
          {
            float d = _robots[j]->get_lasers()[i].get_dist();
            float range = _robots[j]->get_lasers()[i].get_range();
            float value_laser = 0;
            float radius = _robots[j]->get_radius();
        
            if(d != -1)
            {
              // Getting the input between 0 and 1
              d = d - radius;
              float val = 1 - d/(range - radius);
              value_laser = std::min(1.f, std::max(0.f, val));
            }
        
            float alpha = _robots[j]->get_lasers()[i].get_angle() + _robots[j]->get_pos().theta();
            float xr = cosf(alpha) * r_real * (1 + (value_laser*4)) + _map->pixel_to_real(x);
            float yr = sinf(alpha) * r_real * (1 + (value_laser*4)) + _map->pixel_to_real(y);
        
            int xr2 = _map->real_to_pixel(xr);
            int yr2 = _map->real_to_pixel(yr);
        
            unsigned x_laser2 = _map->real_to_pixel(_map->pixel_to_real(x)
                           + _robots[j]->get_lasers()[i].get_gap_dist() 
                           * cosf(_robots[j]->get_pos().theta() 
                            + _robots[j]->get_lasers()[i].get_gap_angle()));
            unsigned y_laser2 = _map->real_to_pixel(_map->pixel_to_real(y)
                           + _robots[j]->get_lasers()[i].get_gap_dist() 
                           * sinf(_robots[j]->get_pos().theta()
                            + _robots[j]->get_lasers()[i].get_gap_angle()));

            _line(_screen, x_laser2, y_laser2,
                  xr2,
                  yr2,
                  0xFF00000);
          }
        }
        
        if(j == _selected)
        {
          // Direction of the robot
          float alpha = _robots[j]->get_pos().theta();
          float xr = cosf(alpha) * _map->pixel_to_real(r) + _map->pixel_to_real(x);
          float yr = sinf(alpha) * _map->pixel_to_real(r) + _map->pixel_to_real(y);
      
          int xr2 = _map->real_to_pixel(xr);
          int yr2 = _map->real_to_pixel(yr);

          _line(_screen, x, y, xr2, yr2, 0x00000FF);
        }
      }
    }
  }

  void Display :: _disp_light_sensors()
  {
    for(std::vector<fastsim_multi::Robot*>::const_iterator it = _robots.begin(); it != _robots.end(); ++it) 
    {
      for (size_t i = 0; i < (*it)->get_light_sensors().size(); ++i)
      {
        const LightSensor& ls = (*it)->get_light_sensors()[i];
        unsigned x_ls = _map->real_to_pixel((*it)->get_pos().x());
        unsigned y_ls = _map->real_to_pixel((*it)->get_pos().y());
        unsigned x_ls1 = _map->real_to_pixel((*it)->get_pos().x() 
                       + 200./(float)ls.get_color() 
                       * cosf((*it)->get_pos().theta() 
                        + ls.get_angle()-ls.get_range()/2.0));
        unsigned y_ls1 = _map->real_to_pixel((*it)->get_pos().y() 
                       + 200./(float)ls.get_color()
                       * sinf((*it)->get_pos().theta()
                        + ls.get_angle()-ls.get_range()/2.0));  
        _line(_screen, x_ls, y_ls, x_ls1, y_ls1, _color_from_id(_screen, ls.get_color()));
        unsigned x_ls2 = _map->real_to_pixel((*it)->get_pos().x() 
                       + 200./(float)ls.get_color() 
                       * cosf((*it)->get_pos().theta() 
                        + ls.get_angle()+ls.get_range()/2.0));
        unsigned y_ls2 = _map->real_to_pixel((*it)->get_pos().y() 
                       + 200./(float)ls.get_color() 
                       * sinf((*it)->get_pos().theta()
                        + ls.get_angle()+ls.get_range()/2.0));  
        _line(_screen, x_ls, y_ls, x_ls2, y_ls2, _color_from_id(_screen, ls.get_color()));
        _line(_screen, x_ls1, y_ls1, x_ls2, y_ls2, _color_from_id(_screen, ls.get_color()));

        if (ls.get_activated())
        {
            const IlluminatedSwitch& is = 
              *_map->get_illuminated_switches()[ls.get_num()];
            unsigned x_is = _map->real_to_pixel(is.get_x());
            unsigned y_is = _map->real_to_pixel(is.get_y());
            _line(_screen, x_ls, y_ls, x_is, y_is, _color_from_id(_screen, is.get_color()));
        }
      }
    }
  }

  void Display :: _disp_camera()
  {
    for(int j = 0; j < _robots.size(); ++j)
    {
      if (!_robots[j]->is_using_camera())
        return;
      unsigned x_ls = _map->real_to_pixel(_robots[j]->get_pos().x());
      unsigned y_ls = _map->real_to_pixel(_robots[j]->get_pos().y());
      float a1 = _robots[j]->get_pos().theta() + _robots[j]->get_camera().get_angular_range() / 2.0;
      
      float distance = 200;
      unsigned x_ls2 = cos(a1) * distance + x_ls;
      unsigned y_ls2 = sin(a1) * distance + y_ls;
      
      _line(_screen, x_ls, y_ls, x_ls2, y_ls2, 0xff0000);

      float a2 = _robots[j]->get_pos().theta() - _robots[j]->get_camera().get_angular_range() / 2.0;
      x_ls2 = cos(a2) * distance + x_ls;
      y_ls2 = sin(a2) * distance + y_ls;
      
      _line(_screen, x_ls, y_ls, x_ls2, y_ls2, 0xff0000);

      // Uncomment if you want to display the camera's rays
      // float _angular_range = M_PI/2;
     // float inc = _angular_range / (_robots[j]->get_camera().pixels().size() - 1);
     // float r = -_angular_range / 2.0f;
     // for (size_t i = 0; i < _robots[j]->get_camera().pixels().size(); ++i, r += inc)
     // {
     //   float alpha = r + _robots[j]->get_pos().theta();
     //   float xr = cos(alpha) * 10000 + _robots[j]->get_pos().x();
     //   float yr = sin(alpha) * 10000 + _robots[j]->get_pos().x();
     //   assert(i < _robots[j]->get_camera().pixels().size());
     //   float xr2 = _map->real_to_pixel(xr);
     //   float yr2 = _map->real_to_pixel(yr);
        
     //   _line(_screen, x_ls, y_ls, xr2, yr2, 0x0000ff);
     // }
    
      // We only display what the camera sees for the selected robot
      if(_display_sensors && (j == _selected))
      {
        static const int pw = _w/_robots[j]->get_camera().pixels().size();
        static const int ph = 40*(_h/800.0f);
        
        // Bias so the rectangle is well centered
        static const int bias = (_w - (_robots[j]->get_camera().pixels().size()) * pw)/2;
        for (size_t i = 0; i < _robots[j]->get_camera().pixels().size(); ++i)
        {
          int pix = _robots[j]->get_camera().pixels()[i]; 

          Uint32 color = pix == -1 ? 0xffffffff : _color_from_id(_screen, pix);
          SDL_Rect r; 
          r.x = i*pw + bias + _w; 
          r.y = 0; 
          r.w = pw; 
          r.h = ph;
          SDL_FillRect(_screen, &r, color);
          _line(_screen, r.x, r.y, r.x, r.y + ph, 0x000000);
        }
        
        // Areas not covered by pixels of the camera
        SDL_Rect r;
        r.x = _w;
        r.y = 0;
        r.w = bias;
        r.h = ph;
        SDL_FillRect(_screen, &r, 0x000000);

        r.x = _robots[j]->get_camera().pixels().size()*pw + bias + _w;
        r.y = 0;
        r.w = bias;
        r.h = ph;
        SDL_FillRect(_screen, &r, 0x000000);
      }
    }
  }

  void Display :: update()
  {
    _events();

    
    // erase robots
    for(std::vector<SDL_Rect>::iterator it = _prev_bb.begin(); it != _prev_bb.end(); ++it)
      SDL_BlitSurface(_map_bmp, &(*it), _screen, &(*it));
    
    // erase all
    SDL_BlitSurface(_map_bmp, 0, _screen, 0);

    // goals
    _disp_goals();
    
    // illuminated switches
    _disp_switches();
    
    // light sensor
    _disp_light_sensors();

    // radars
    _disp_radars();
    
    // camera
    _disp_camera();

    // lasers
    _disp_lasers();

    //for(std::vector<fastsim_multi::Robot>::const_iterator it = _robots.begin(); it != _robots.end(); ++it) 
    bool first_hunter = true;
    for(int i = 0; i < _robots.size(); ++i)
    {
      // convert to pixel
      unsigned x = _map->real_to_pixel(_robots[i]->get_pos().x());
      unsigned y = _map->real_to_pixel(_robots[i]->get_pos().y());
      unsigned r = _map->real_to_pixel(_robots[i]->get_radius());
      float theta = _robots[i]->get_pos().theta();

      // draw the circle again (robot)
      unsigned int col = _robots[i]->display_color();
      _disc(_screen, x, y, r, _color_from_id(_screen,col));

      if(i == _selected)
      {
        _circle(_screen, x, y, r, _color_from_id(_screen, 9));
        _circle(_screen, x, y, r - 2, _color_from_id(_screen, 9));
      }
      
      // direction
      Uint32 color = SDL_MapRGB(_screen->format, 0, 255, 0);
      _line(_screen, x, y, (int) (r * cosf(theta) + x), (int)(r * sinf(theta) + y), color);

      // Save the trajectory of each robot
      struct Position pos_traj;
      pos_traj.x = x;
      pos_traj.y = y;
      pos_traj.radius = 1;
      pos_traj.color = col;
      _trajectory_log[i].push_back(pos_traj);
    }
    
    // Starting positions
    if(_start_positions.empty())
    {
      for(int i = 0; i < _robots.size(); ++i)
      {
        unsigned int col = _robots[i]->display_color();
        struct Position pos;
        pos.x = _robots[i]->get_pos().x();
        pos.y = _robots[i]->get_pos().y();
        _start_positions.push_back(pos);
      }
    }

    // bumpers
    _disp_bumpers();
        
    for(int i = 0; i < _robots.size(); ++i)
    {        
      SDL_Rect rect;
      _bb_to_sdl(_robots[i]->get_bb(), &rect);
      using namespace std;
      rect.x = max(0, min((int)rect.x, (int)_prev_bb[i].x));
      rect.y = max(0, min((int)rect.y, (int)_prev_bb[i].y));
      rect.w = max(rect.w, _prev_bb[i].w);
      rect.h = max(rect.h, _prev_bb[i].h);
      
      if (rect.x + rect.w > _w) rect.w = _w;
      if (rect.y + rect.h > _h) rect.h = _h;
      
      if(!_no_video)
      {
        // the fast one
        SDL_UpdateRect(_screen, rect.x, rect.y, rect.w, rect.h);
        // the slow one
        SDL_UpdateRect(_screen, 0, 0, _screen->w, _screen->h);
      }

      _bb_to_sdl(_robots[i]->get_bb(), &_prev_bb[i]);
    }
  }
  
  void Display :: dump_behaviour_log(const char * file)
  {
    SDL_Surface* behaviour_log = SDL_ConvertSurface(_map_bmp_no_sensors, _map_bmp_no_sensors->format, SDL_SWSURFACE);

    // Trajectories drawing
    for(size_t i = 0; i < _trajectory_log.size(); ++i)
    {
      for(size_t j = 0; j < _trajectory_log[i].size(); ++j)
      {
        int color = _color_from_id(behaviour_log, _trajectory_log[i][j].color);
        _disc(behaviour_log, _trajectory_log[i][j].x, _trajectory_log[i][j].y, _trajectory_log[i][j].radius, color);
      }
    }

    _trajectory_log.clear();
    _trajectory_log.resize(2);

    // Starting positions drawing
    for(size_t i = 0; i < _start_positions.size(); ++i)
    {
      // Starting position
      float x = _start_positions[i].x;
      float y = _start_positions[i].y;
      
      unsigned _x = _map->real_to_pixel(x);
      unsigned _y = _map->real_to_pixel(y);
      unsigned _r = _map->real_to_pixel(5);

      Uint32 _color = SDL_MapRGB(behaviour_log->format, 0, 0, 0);
      _disc(behaviour_log, _x, _y, _r, _color);
    }
    _start_positions.clear();

    // --- Here you can log anything you want (e.g. radars, switches...) ---

    SDL_SaveBMP(behaviour_log, file);
  }
}

#endif
