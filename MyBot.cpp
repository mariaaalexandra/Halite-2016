#include <time.h>
#include <ctime>
#include <fstream>
#include <map>
#include <cstdlib>
#include <stdlib.h>
#include <time.h>
#include <set>
#include <cmath>
#include <chrono>

#include "hlt.hpp"
#include "networking.hpp"

using namespace hlt;

#define CAP_BOUND 265
#define MAX_CARDINAL 4

typedef struct __attribute__((packed))
{
    Site curr_site;
    Location curr_location;
} dot_info;

typedef struct __attribute__((packed))
{
    float production;
    float strength;
} site_attributes;

typedef struct __attribute__((packed))
{
    unsigned char width;
    unsigned char height;
} map_info;

typedef struct __attribute__((packed))
{
    int b;
    int a;
} int_pair;

typedef struct __attribute__((packed))
{
    float dist;
    int myArea;
} surface_info;

typedef struct __attribute__((packed))
{   Location start_loc;
    Location goal_loc;
} changing_loc;

namespace helper {
    bool is_not_my_id(int x, int y) {
        if (x != y) {
            return true;
        }
        return false;
    }
    bool less_than_max_lim(int a, int b) {
        if(a + b < 255) {
            return true;
        }
        return false;
    }
    void update_max_val(float current, float max, dot_info &neigh, dot_info &best) {
        if (current > max) {
            max = current;
            best.curr_location = neigh.curr_location;
            best.curr_site = neigh.curr_site;
        }
    }
};

namespace direction {

    void find_move(Site site, int move_found, int direction) {
        switch(site.strength < 5 * site.production) {
            case true:
            direction = 0;
            move_found = 1;
            break;
            case false:
            break;
        }
    }
}

namespace site {
    site_attributes get_info_site(float prod, float strength) {
        site_attributes new_atr;
        new_atr.production = prod;
        new_atr.strength = strength;
        return new_atr;
    }
}

namespace surface {
    surface_info get_surface(float dist, int myArea) {
        surface_info new_surface;
        new_surface.dist = dist;
        new_surface.myArea = myArea;
        return new_surface;
    }    
}

namespace map {
    //compare dimensions of a map
    int compare_dim(GameMap map) {
        int width = map.width;
        int height = map.height;
        //1 for less
        if (width < height) {
            return 1;
        }
        //2 for more
        else if (width > height) {
            return 2;
        }
        //3 for equality
        return 0;
    }
}

namespace force {

    float force_calculation(float strength, float production, int num_neighbors, float neighbor_force, float dist) {
        float force = 0.0;
        if (strength) {
            force = production / strength;
        }
        else {
            force = production;
        }
        force = (force / (dist * dist * dist * 1) + neighbor_force) / (num_neighbors + 1);
        return force;
    }

    float first_case(dot_info current_loc, GameMap &present_map, Location original_location, int my_id, float dist) {
        float force = 0.0;
        site_attributes site_atr = site::get_info_site((float)current_loc.curr_site.production, (float)current_loc.curr_site.strength);
        int num_neighbors = 0;
        float neighbor_force = 0.0;
        int i = 0;
        while (i != 4) {
             unsigned char dir = i + 1;
            dot_info neighbor_info;
            neighbor_info.curr_location = present_map.getLocation(current_loc.curr_location, dir);
            neighbor_info.curr_site = present_map.getSite(neighbor_info.curr_location);
            if(helper :: is_not_my_id (neighbor_info.curr_site.owner, my_id))
            {   site_attributes neighbor_atr = site::get_info_site((float)neighbor_info.curr_site.production, (float)neighbor_info.curr_site.strength);
                float neighbor_dist = present_map.getDistance(original_location, neighbor_info.curr_location);
                float neighbor_val = (neighbor_atr.strength ? (neighbor_atr.production / neighbor_atr.production) : neighbor_atr.production) / (neighbor_dist * neighbor_dist * neighbor_dist * 1);
                neighbor_force += neighbor_val;
                num_neighbors++;
            }
            i++;
        }
        force = force :: force_calculation(site_atr.strength, site_atr.production, num_neighbors, neighbor_force, dist); 
        return force;
    }

    float second_case(dot_info current_loc, Location original_location, int my_id, float dist, GameMap &present_map) {
        int i = 0;
        float force = 0.0;
        while (i != 4) {
            unsigned char dir = i + 1;
            dot_info neighbor_loc;
            neighbor_loc.curr_location = present_map.getLocation(current_loc.curr_location, dir);
            neighbor_loc.curr_site = present_map.getSite(neighbor_loc.curr_location);
            if((neighbor_loc.curr_site.owner != 0) && (neighbor_loc.curr_site.owner != my_id))
            {
                float neighbor_dist = present_map.getDistance(neighbor_loc.curr_location, original_location);
                float strength = (float) neighbor_loc.curr_site.strength / (neighbor_dist * neighbor_dist * neighbor_dist);
                force += strength;
            }
            i++;
        }
        return force;
    }
    
}

namespace dot {
    dot_info get_dot(int_pair coords, GameMap& presentMap)
    {
        dot_info my_dot;
        my_dot.curr_location.x = coords.b;
        my_dot.curr_location.y = coords.a;
        my_dot.curr_site = presentMap.getSite(presentMap.getLocation({my_dot.curr_location.x, my_dot.curr_location.y}, (unsigned char) STILL));
        return my_dot;
    }

    dot_info get_dot(int_pair coords, GameMap& presentMap, const int DIRECTION = STILL)
    {
        dot_info my_dot;
        my_dot.curr_location = presentMap.getLocation({(unsigned char) coords.b,  (unsigned char) coords.a}, DIRECTION);
        my_dot.curr_site = presentMap.getSite(my_dot.curr_location);
        return my_dot;
    }

    dot_info get_dot(Location loc, Site site) {
        dot_info my_dot;
        my_dot.curr_location = loc;
        my_dot.curr_site = site;
        return my_dot;
    }
};

namespace coords {
    bool equal_coords(int x1, int y1, int x2, int y2) {
        switch (x1 == x2 && y2 == y1) { 
            case 1:
                return true;
            case 0:
                return false;
        }
    }
};

namespace target {
  
    int start_is_less(GameMap present_map,Location &start_location, Location &goal_location, std::map<Location, int> &reserved_enemy, Site start_site, Site goal_site) {
        switch(start_site.strength <= goal_site.strength) {
            case true : 
                return 0; // return STILL direction
            case false :
                int i = 0;
                while (i != 4) {
                    unsigned char curr_dir = i + 1;
                    Location curr_location = present_map.getLocation(start_location, curr_dir);
                    bool logic = coords:: equal_coords(goal_location.x, goal_location.y, curr_location.x, curr_location.y);
                    switch (logic == true && (reserved_enemy[goal_location] + start_site.strength < 255)) {
                        case 1:
                            return curr_dir;
                    }
                    i++;
                }
            return 0;//return STILL
        }
    }

    void switch_for_optimal_dist(float goal_dist, float min_dist, dot_info current, int best_dir, int curr_dir, Location &goal_location,
                    GameMap &present_map) {
        float curr_dist = present_map.getDistance(current.curr_location, goal_location);
        if(curr_dist < goal_dist) {
           dot_info best_dot = dot::get_dot(current.curr_location, current.curr_site);
           min_dist = curr_dist;
           best_dir = curr_dir;

        }
    }

    void dir_is_not_STILL(std::map<Location, int> &reserved_own, dot_info best_dot, Location start, GameMap &present_map) {
        reserved_own[start] = 0;
        Site start_site = present_map.getSite(start);
        switch(reserved_own.count(best_dot.curr_location)) {
            case 0:
            reserved_own[best_dot.curr_location] = best_dot.curr_site.strength + start_site.strength;
            case 1:
            reserved_own[best_dot.curr_location] = start_site.strength + reserved_own[best_dot.curr_location];
        }

    }
}

unsigned char get_nearest_direction(dot_info current, GameMap &present_map, map_info map_dimensions, unsigned char my_id)
{
    //find max distance
    unsigned char max_dist;

    //compare dimensions of the map
    switch(map::compare_dim(present_map)) {
        case 1:
        max_dist = map_dimensions.width / 2;
        case 0:
        max_dist = map_dimensions.height / 2;
        case 2:
        max_dist = map_dimensions.height / 2;
    }

    //find best direction
    unsigned char best_dir = SOUTH;

    //4 directions
    int i = 0;
    while(i != MAX_CARDINAL) {
        //current distance
        unsigned char dist = 0;
        //current direction
        unsigned char curr_direction = i + 1;
        Location curr_location = current.curr_location;
        Site curr_site = current.curr_site;

        dot_info new_dot = dot::get_dot(current.curr_location,current.curr_site);

        for (; dist < max_dist;) {
            if (helper::is_not_my_id(new_dot.curr_site.owner, my_id) == false) {
                new_dot.curr_location = present_map.getLocation(new_dot.curr_location, curr_direction);
                new_dot.curr_site = present_map.getSite(new_dot.curr_location);
                dist++;
            }
        }

        if(dist < max_dist)
        {
            max_dist = dist;
            best_dir = curr_direction;
        }
        i++;
    }
    return best_dir;
}

float compute_force (Location original_location, dot_info current_loc, GameMap present_map, surface_info surf, int my_id)
{
    float force = 0.0;
    if(current_loc.curr_site.owner == 0)
    {
        force = force::first_case(current_loc,present_map, original_location, my_id, surf.dist);
    }
    else//that is, if owner is not in (0, myID)
    {
        force = force :: second_case(current_loc,original_location,my_id,surf.dist,present_map);
    }
	return force;
}

float heuristic(dot_info current, GameMap &present_map,  unsigned char &my_id)
{
    switch((current.curr_site.owner == 0) && (current.curr_site.strength > 0)) {
        case 1: 
            return static_cast<float>(current.curr_site.production) / current.curr_site.strength;
            break;

        case 0:
            {float strength;
            //putere 0 si e neocupat
            if(current.curr_site.owner == 0)
            {
                strength = current.curr_site.production;
            }
            //este ocupat 
            else
            {
                strength = 0.0;
            }

            int i = 0;
            while (i != 4) {
                int direction = i + 1;
                Site site = present_map.getSite(current.curr_location, direction);
                if (site.owner != 0) {
                    if (helper::is_not_my_id(site.owner, my_id)) {
                        strength += site.strength;
                    }
                }
                i++;
            }
            return strength;
            break;
            }
    }
    
}

bool is_on_borderPosition(dot_info position, GameMap present_map,  unsigned char my_id)
{   
    int i = 0;
    do {
        unsigned char direction = i + 1;
        Site curr_site = present_map.getSite(position.curr_location, direction);
        bool logic = helper::is_not_my_id(curr_site.owner, my_id);
        if (logic == true) {
            return true;
        }
        i++;
    }
    while (i != 4);

    return false;
    
}

bool is_on_borderLocation(Location &location, GameMap present_map,  unsigned char my_id)
{   
    int i = 0;
    do {
        unsigned char direction = i + 1;
        Site curr_site = present_map.getSite(location, direction);
        bool logic = helper::is_not_my_id(curr_site.owner, my_id);
        if (logic == true) {
            return true;
        }
        i++;
    }
    while (i != 4);

    return false;
    
}

Location get_best_target_on_border_location(GameMap &present_map, unsigned char my_id)
{
    float max_val = -1.0;
    dot_info best_dot;
    unsigned short a = 0;
    while(a < present_map.height) {
        unsigned short b = 0;
        while(b < present_map.width) {
    
            dot_info current_dot = dot::get_dot({b,a}, present_map, STILL);
            if(!helper::is_not_my_id(current_dot.curr_site.owner,my_id))
            {
                int i = 0;
                do {
                    unsigned char neighbor_direction = i + 1;
                    dot_info neighbor = dot::get_dot({current_dot.curr_location.x, current_dot.curr_location.y}, present_map, neighbor_direction);
                    current_dot.curr_location = neighbor.curr_location;
                    current_dot.curr_site = neighbor.curr_site;

                    if(helper::is_not_my_id(neighbor.curr_site.owner,my_id))
                    {
                        float curr_val = heuristic(current_dot, present_map, my_id);
                        helper::update_max_val(curr_val,max_val,neighbor,best_dot);
                    }
                    i++;
                }while(i < 4);
            }
            b++;
        }
        
        a++;

    }
   
    return best_dot.curr_location;
}

unsigned char get_best_target_on_border_direction(changing_loc &start_to_go,
                    GameMap &present_map, unsigned char my_id, std::map<Location, int> &reserved_own,
                    std::map<Location, int> &reserved_enemy)
{

    Site start_site = present_map.getSite(start_to_go.start_loc);
    Site goal_site = present_map.getSite(start_to_go.goal_loc);
    float dist_start_goal = present_map.getDistance(start_to_go.start_loc, start_to_go.goal_loc);
    
    if((fabs(dist_start_goal - 1.0) < 0.01))
    {
        Location copy_st = start_to_go.start_loc;
        Location copy_goal = start_to_go.goal_loc;
            
        target::start_is_less(present_map,copy_st,copy_goal,reserved_enemy,start_site,goal_site);
    }

    dot_info best_loc;
    best_loc.curr_location = start_to_go.start_loc;
    best_loc.curr_site = start_site;

    unsigned char best_direction = 0;
    float min_distance = dist_start_goal;
    
    int i = 0;
   
    while (i != 4)
    {
        unsigned char curr_direction = i + 1;
        dot_info curr_loc;
        curr_loc.curr_location = present_map.getLocation(start_to_go.start_loc, curr_direction);
        curr_loc.curr_site = present_map.getSite(curr_loc.curr_location);


        if((helper::is_not_my_id(curr_loc.curr_site.owner, my_id) == false) && is_on_borderPosition(curr_loc, present_map, my_id))
        {
            if (helper ::less_than_max_lim(reserved_own[curr_loc.curr_location], start_site.strength) == true) 
            {
                Location copy_st = start_to_go.start_loc;
                Location copy_goal = start_to_go.goal_loc;
            
                target::switch_for_optimal_dist(dist_start_goal, min_distance, curr_loc, best_direction, curr_direction, copy_goal, present_map);
            }
        }
        i++;

    }

    if(best_direction > 0)
    {
      
        target::dir_is_not_STILL(reserved_own,best_loc, start_to_go.start_loc, present_map);
    }
   


    return best_direction;
}

int main ()
{
	unsigned char myID;
	GameMap currMap;
	std::set<Move> moveList;
	getInit (myID, currMap);
	sendInit ("ItWorksOnMyPC¯\\_(ツ)_/¯");

	std::map<Move, bool> prevMap;
	std::map<Location, int> reserved_own;
	std::map<Location, int> reserved_enemy;
	while(true)
	{
	   
		moveList.clear ();
		getFrame (currMap);
       
        std::chrono::duration<double, std::milli> think_time = std::chrono::milliseconds(0);
		Location best_target_on_border_location = get_best_target_on_border_location(currMap, myID);
		

		std::multiset<Location> border;
		int myArea = 0;
        for (unsigned short a = 0; a < currMap.height; a ++) {
            
            for (unsigned short b = 0; b < currMap.width; b ++) {
                Site curr_site = currMap.getSite ({b, a}, STILL);
               
              
				if (curr_site.owner == myID)
                {
                    myArea ++;
					reserved_own[{b, a}] = curr_site.strength;
					
                }
				else
				{
					for (unsigned char i = 1; i < 5; i ++)
						if (currMap.getSite ({b, a}, i).owner == myID)
						{
							border.insert ({b, a});
							break;
						}
                    reserved_enemy[{b, a}] = 0;
                       
				}
            }
        }
       
		
       for (unsigned short a = 0; a < currMap.height; a ++)
		{   
			for (unsigned short b = 0; b < currMap.width; b ++)
			{
			    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
                dot_info dot_atributes = dot::get_dot({b,a},currMap.getSite({b,a}, STILL));

			    unsigned char direction;
				if (dot_atributes.curr_site.owner == myID)
				{
				    float force_x = 0.0;
                    float force_y = 0.0;
                    int move_found = 0;
                    Location my_loc = dot_atributes.curr_location;
				    if(!is_on_borderLocation(my_loc, currMap, myID))
                    {
                        if(think_time > std::chrono::milliseconds(900))
                        {
                            dot_info current_location;
                            current_location.curr_location = dot_atributes.curr_location;
                            current_location.curr_site = dot_atributes.curr_site;

                            map_info current_map;
                            current_map.height = currMap.height;
                            current_map.width = currMap.width;

                            direction = get_nearest_direction(current_location, currMap, current_map, myID);
                            reserved_own[currMap.getLocation(dot_atributes.curr_location, direction)] += dot_atributes.curr_site.strength;
                            reserved_own[dot_atributes.curr_location] -= dot_atributes.curr_site.strength;
                            move_found = 1;
                        }
                        else
                        {
                        
                            for (std::set<Location>::iterator it = border.begin (); it != border.end (); it++)
                            {   dot_info enemy_loc;
                                enemy_loc.curr_location = *it;
                                enemy_loc.curr_site = currMap.getSite (*it, STILL);
                               
                               
                                float dist = currMap.getDistance(dot_atributes.curr_location, enemy_loc.curr_location);
                                float angle = currMap.getAngle(dot_atributes.curr_location, enemy_loc.curr_location);
                                surface_info surface = surface::get_surface(dist, myArea);
                                float force = compute_force(dot_atributes.curr_location, enemy_loc, currMap, surface, myID);
                                force_x += force * cos(angle);
                                force_y += force * sin(angle);
                                
                            }
                            direction = STILL;

                            int logic = 0;
                            switch ((dot_atributes.curr_site.strength + reserved_own[currMap.getLocation(dot_atributes.curr_location, 2)] < CAP_BOUND) 
                            && force_x + force_y > 0 && force_x - force_y > 0) {
                                case true:
                                direction = EAST;
                                logic = 1;
                        
                                break;
                                case false:
                                    switch((dot_atributes.curr_site.strength + reserved_own[currMap.getLocation(dot_atributes.curr_location, 3)] < CAP_BOUND) 
                            && force_x + force_y > 0 && force_x - force_y < 0) {
                                    case true:
                                    direction = SOUTH;
                                    logic = 1;
                         
                                    break;
                                        case false:
                                            switch((dot_atributes.curr_site.strength + reserved_own[currMap.getLocation({b, a}, 4)] < CAP_BOUND) 
                                        && force_x + force_y < 0 && force_x - force_y < 0) {
                                            case true:
                                            direction = WEST;
                                            logic = 1;
                            
                                            break;
                                            case false:
                                                if((dot_atributes.curr_site.strength + reserved_own[currMap.getLocation({b, a}, 1)] < CAP_BOUND) 
                                            && force_x + force_y < 0 && force_x - force_y > 0) {
                                                direction = NORTH;
                                                logic = 1;
                                               
                                            }
                                    }
                                }
                                

                            }
                            if (logic == 1) {
                                //try fun for 2 apeluri
                                reserved_own[dot_atributes.curr_location] -= dot_atributes.curr_site.strength;
                                reserved_own[currMap.getLocation(dot_atributes.curr_location, direction)] += dot_atributes.curr_site.strength;
                                move_found = 1;
                            }
                           
                        }
                        //functie
                        if(dot_atributes.curr_site.strength < 5 * dot_atributes.curr_site.production)
                        {
                            
                            if(move_found == 1)
                            {
                                reserved_own[currMap.getLocation(dot_atributes.curr_location, direction)] -= dot_atributes.curr_site.strength;
                                reserved_own[dot_atributes.curr_location] += dot_atributes.curr_site.strength;
                            }
                            direction = STILL;
                            move_found = 1;

                        }
                    }

                    else//that is if our square is on border
                    {
                    
                        int move_found = 0;
                        double max_val = -1.0;
                        direction = 123;
                        Site best_site;
                        Location best_location;
                        
                        for(int i = 0; i < 4; i++)
                        {
                            unsigned char curr_direction = CARDINALS[i];
                            Location curr_location = currMap.getLocation(dot_atributes.curr_location, curr_direction);
                            Site curr_site = currMap.getSite(curr_location);
                            dot_info current_location;
                            current_location.curr_location = curr_location;
                            current_location.curr_site = curr_site;

                            if(curr_site.owner != myID)
                            {
                               
                                float curr_val = heuristic(current_location, currMap, myID);
                          
                                if((curr_val > max_val) && (reserved_enemy[curr_location] + dot_atributes.curr_site.strength < CAP_BOUND))
                                {
                                    max_val = curr_val;
                                    direction = curr_direction;
                                    best_site = curr_site;
                                    best_location = curr_location;
                                
                                }
                            }

                        }
                        
                        if((dot_atributes.curr_site.strength > best_site.strength) && (reserved_enemy[best_location] + dot_atributes.curr_site.strength < CAP_BOUND))
                        {
                            if (reserved_enemy.count(best_location) == 0) {
                                reserved_enemy[best_location] = dot_atributes.curr_site.strength;
                            }
                            else {
                                reserved_enemy[best_location] += dot_atributes.curr_site.strength;
                            }
                            reserved_own[dot_atributes.curr_location] = 0;
                            move_found = 1;
                        }

                        direction::find_move(dot_atributes.curr_site,move_found, direction);
                      
                        if(move_found == 0)
                        {
                            changing_loc start_go;
                            start_go.goal_loc = best_target_on_border_location;
                            start_go.start_loc = dot_atributes.curr_location;
                            direction = get_best_target_on_border_direction(start_go, currMap,
                                                   myID, reserved_own, reserved_enemy);
                        }
                        
                    }


					Move move = {{b, a}, direction};
					moveList.insert (move);
				}
				std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
				think_time += (end - start);
              
			}
           
		}

		sendFrame (moveList);
	}

	return 0;
}