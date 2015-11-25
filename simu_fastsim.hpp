#ifndef SIMU_FASTSIM_HPP_
#define SIMU_FASTSIM_HPP_

#include <sferes/simu/simu.hpp>
#include <sferes/misc/rand.hpp>
#include "fastsim.hpp"

namespace sferes
{
  namespace simu
  {
    SFERES_SIMU(Fastsim, Simu)
    {
      public:
        Fastsim()
        {
          _map = boost::shared_ptr<fastsim_multi::Map>(new fastsim_multi::Map(Params::simu::map_name(), 600.0f));
        }

        Fastsim(const fastsim_multi::Map & m)
        {
          _map = boost::shared_ptr<fastsim_multi::Map>(new fastsim_multi::Map(m));
        }

        ~Fastsim()
        {
          for(int i = 0; i < _robots.size(); ++i)
            delete _robots[i];
          _robots.clear();
        }

        void init()
        {}
        void reinit()
        {
          for(int i = 0; i < _robots.size(); ++i)
            delete _robots[i];
          _robots.clear();
        }

        void refresh() 
        { 
          // We don't want to always refresh the robots in the same order each time
          std::vector<size_t> ord_vect;
          sferes::misc::rand_ind(ord_vect, _robots.size());
          
          for(int i = 0; i < ord_vect.size(); ++i)
          {
            int num = (int)ord_vect[i];
            assert(num < _robots.size());
            _map->update(_robots[num]->get_pos()); 
          }
        }

        void init_view(bool no_video = false, bool dump = false)
        {
          _display =
            boost::shared_ptr<fastsim_multi::Display>(new fastsim_multi::Display(_map, _robots, true, no_video));
        }
        void init_robots_view()
        {
          if(_display)
            _display->init_robots();
        }
        void set_map(const boost::shared_ptr<fastsim_multi::Map>& map) { _map = map; }
        void refresh_view()
        {
          _display->update();
        }
        void refresh_map_view()
        {
          _display->update_map();
        }
        void switch_map()
        {
          _map->terrain_switch(Params::simu::alt_map_name());
        }
        void reset_map()
        {
          _map->terrain_switch(Params::simu::map_name());
        }
        void move_robot(float v1, float v2, int index) 
        {
          assert(index < _robots.size());
          _robots[index]->move(v1, v2, _map);
        }
        void teleport_robot(float x, float y, int index) 
        {
          assert(index < _robots.size());
          _robots[index]->teleport(x, y, _map);
        }

        void add_robot(fastsim_multi::Robot *robot)
        {
          _robots.push_back(robot);

          // We add the robot to the map
          _map->add_robot(robot);
        }

        void remove_robot(int index)
        {
          assert(index < _robots.size());
          _map->remove_robot(index);
          _robots.erase(_robots.begin() + index);
        }

        void change_selected_robot(int selected)
        {
          assert(selected < _robots.size());
          _display->set_selected(selected);
        }

        boost::shared_ptr<fastsim_multi::Map> map() { return _map; }
        const boost::shared_ptr<fastsim_multi::Map> map() const { return _map; }

        std::vector<fastsim_multi::Robot*>& robots() { return _robots; }
        const std::vector<fastsim_multi::Robot*>& robots() const { return _robots; }

        fastsim_multi::Display& display() {return *_display; }
      protected:
        std::vector<fastsim_multi::Robot*> _robots;
        boost::shared_ptr<fastsim_multi::Map> _map;
        boost::shared_ptr<fastsim_multi::Display> _display;
    };
  }
}

#endif
