#include <iostream>
#include "linear_camera.hpp"


namespace fastsim_multi
{
  void LinearCamera :: update(const Posture& pos,
                             const boost::shared_ptr<Map>& map)
  {
    float inc = _angular_range / (_pixels.size() - 1);
    float r = -_angular_range / 2.0f;
    for (size_t i = 0; i < _pixels.size(); ++i, r += inc)
    {
    	float alpha = r + pos.theta();
    	float xr = cos(alpha) * _range + pos.x();//range = 10000
    	float yr = sin(alpha) * _range + pos.x();
    	assert(i < _pixels.size());
    	Map::Return_check_t ret = map->check_inter_is_dist(pos.x(), pos.y(), xr, yr);
      _pixels[i] = ret.pixel;
      _dist[i] = ret.dist;
    }
  }
}
