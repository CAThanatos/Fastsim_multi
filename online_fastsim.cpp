#include <boost/foreach.hpp>
#include "simu_fastsim.hpp"

using namespace sferes;

// -- Parameters --
struct Params
{
  struct simu
  {
    SFERES_STRING(map_name, "modules/fastsim_multi/cuisine.pbm");
    SFERES_CONST int nb_steps = 20000;
    SFERES_CONST int video_frame = 50;
  };
};



// -- Main program --
int main(int argc, char *argv[])
{
  using namespace fastsim_multi;
  typedef simu::Fastsim<Params> simu_t;
  simu_t s;

  bool display = false;

  // Program arguments
  if (argc > 1)
  {
    int cpt = 1;
    while (cpt < argc)
    {
      std::string arg = (std::string)argv[cpt];

      // We want to have a display
      if(0 == arg.compare("-d"))
        display = true;

      cpt++;
    }
  }

  // -- Initialization of the simulation --
  if(display)
    s.init_view();

  // -- Addition of robots and objects --
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

  if(display)
    s.init_robots_view();

  Map::ill_sw_t s1 = Map::ill_sw_t(new IlluminatedSwitch(1, 10, 300, 251, true));
  s.map()->add_illuminated_switch(s1);


  // -- Evolutionary loop --
  int cpt_frame = 0;
  for(size_t i = 0; i < Params::simu::nb_steps; ++i)
  {
    // Random update of each robot
    std::vector<size_t> ord_vect;

    // List that contains the index of each robot in a random order
    misc::rand_ind(ord_vect, s.robots().size());

    for(size_t j = 0; j < ord_vect.size(); ++j)
    {
      // Current robot we want to update
      Robot* rob = s.robots()[ord_vect[j]];


      // ----------------
      // -- Here you can put anything that concerns robot update (neural network computing for example) --
      // ----------------


      // The update of each of the robot's wheels speed needs to be computed
      float v1 = 0.0;
      float v2 = 0.0;

      // We move the robot
      s.move_robot(v1, v2, ord_vect[j]);
    }

    if(display)
    {
      s.refresh_view();

      // Uncomment if you want to save a picture of the simulation each Params::simu::video_frame step
      // if(i%Params::simu::video_frame == 0)
      // {
      //   std::string file_dump = "testVideo";
      //   file_dump += boost::lexical_cast<std::string>(cpt_frame + 1) + ".bmp";
      //   s.display().save_BMP(file_dump.c_str());
      //   cpt_frame++;
      // }
    }
  }

  return 0;
}
