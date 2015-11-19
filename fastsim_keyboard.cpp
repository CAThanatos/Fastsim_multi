#include <boost/foreach.hpp>
#include "simu_fastsim.hpp"

using namespace sferes;
struct Params
{
  struct simu
  {
    SFERES_STRING(map_name, "modules/fastsim_multi/cuisine.pbm");
  };
};



int main(int argc, char *argv[])
{
  using namespace fastsim_multi;
  typedef simu::Fastsim<Params> simu_t;
  simu_t s;

  // s.init_view(true);
  s.init_view();

  Robot* r1 = new Robot(20.0f, Posture(250, 250, -M_PI/2), 2);
  r1->use_camera();
  r1->add_laser(Laser(M_PI / 3, 50));
  r1->add_laser(Laser(0, 50));
  r1->add_laser(Laser(-M_PI / 3, 50));
  s.add_robot(r1);

  Robot* r2 = new Robot(20.0f, Posture(100, 100, -M_PI/2), 2);
  r2->use_camera();
  r2->add_laser(Laser(M_PI / 3, 50));
  r2->add_laser(Laser(0, 50));
  r2->add_laser(Laser(-M_PI / 3, 50));
  s.add_robot(r2);

  s.init_robots_view();

  Map::ill_sw_t s1 = Map::ill_sw_t(new IlluminatedSwitch(1, 10, 300, 251, true));
  // Map::ill_sw_t s2 = Map::ill_sw_t(new IlluminatedSwitch(2, 10, 530, 150, true));
  // Map::ill_sw_t s3 = Map::ill_sw_t(new IlluminatedSwitch(3, 10, 200, 150, true));
  // Map::ill_sw_t s4 = Map::ill_sw_t(new IlluminatedSwitch(4, 10, 400, 350, true));

  s.map()->add_illuminated_switch(s1);
  // s.map()->add_illuminated_switch(s2);
  // s.map()->add_illuminated_switch(s3);
  // s.map()->add_illuminated_switch(s4);

  int numkey;
  int num_robot = 0;
  int cpt = 0, cpt_frame = 0;
  while(true)
  {
    SDL_PumpEvents();
    Uint8* keys = SDL_GetKeyState(&numkey);

    if(keys[SDLK_TAB])
    {
      num_robot = (num_robot + 1)%(s.robots().size());
      s.change_selected_robot(num_robot);
    }
    if (keys[SDLK_UP])
    	s.move_robot(1.0, 1.0, num_robot);
    if (keys[SDLK_DOWN])
      s.move_robot(-1.0, -1.0, num_robot);
    if (keys[SDLK_LEFT])
     	s.move_robot(1.0, -1.0, num_robot);
    if (keys[SDLK_RIGHT])
     	s.move_robot(-1.0, 1.0, num_robot);
    // s.move_robot(1.0, 1.0, num_robot);
    s.refresh();
    s.refresh_view();

    // if(cpt%1 == 0)
    // {
    //   std::string file_dump = "testVideo";
    //   if((cpt_frame + 1) < 1000)
    //     file_dump += "0";
    
    //   if((cpt_frame + 1) < 100)
    //     file_dump += "0";
    
    //   if((cpt_frame + 1) < 10)
    //     file_dump += "0";
      
    //   file_dump += boost::lexical_cast<std::string>(cpt_frame + 1) + ".bmp";
    //   s.display().save_BMP(file_dump.c_str());
    //   cpt_frame++;
    // }

    // for (size_t j = 0; j < r1->get_lasers().size(); ++j)
    //   std::cout << r1->get_lasers()[j].get_dist() << "/";
    // std::cout << std::endl;
    // for (size_t j = 0; j < r1->get_camera().pixels().size(); ++j)
    //   std::cout << "(" << r1->get_camera().pixels()[j] << ";" << r1->get_camera().dist()[j] << ")/";
    // std::cout << std::endl;

    cpt++;

    if(cpt >= 1000)
      break;
  }

  std::string file_trace = "trace_behaviour.bmp";
  s.display().dump_behaviour_log(file_trace.c_str());

  return 0;
}
