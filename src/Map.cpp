#include "../include/Map.hpp"

Map::Map(){

	obs_map=new int*[MAPX];
	for(int i=0;i<MAPX;i++)
	{
		obs_map[i]=new int[MAPY];
		for(int j=0;j<MAPY;j++)
			obs_map[i][j]=0;
	}

	Mat obsmap=imread("maps/map3.png", 0);

	for(int i=0;i<MAPX;i++)
		for(int j=0;j<MAPY;j++)
			if(obsmap.at<uchar>(j/2,i/2)<=120)
				obs_map[i][j]=1;
			else
				obs_map[i][j]=0;

	cout<<"Cost Map Initialized!"<<endl;
}

void Map::initCollisionChecker(){
	acc_obs_map=new int*[MAPX];
	for(int i=0;i<MAPX;i++)
	{
		acc_obs_map[i]=new int[MAPY];
		for(int j=0;j<MAPY;j++)
			acc_obs_map[i][j]=obs_map[i][j];
	}

	for(int i=0;i<MAPX;i++)
		for(int j=1;j<MAPY;j++)
			acc_obs_map[i][j]=acc_obs_map[i][j-1]+acc_obs_map[i][j];

	for(int j=0;j<MAPY;j++)
		for(int i=1;i<MAPX;i++)
			acc_obs_map[i][j]=acc_obs_map[i-1][j]+acc_obs_map[i][j];

	cout<<"Collision checker initialized!"<<endl;
	return;
}

bool Map::checkCollision(State pos){

	// cout<<"Collision checking: "<<endl;
	// cout<<pos.x<<","<<pos.y<<","<<pos.theta<<endl;
	// cout<<pos.gx<<","<<pos.gy<<","<<pos.gtheta<<endl;

	if(pos.x>=MAPX || pos.x<0 || pos.y>=MAPY || pos.y<0 || pos.theta>=Theta || pos.theta<0)
		return true;

	if(pos.gx>=GX || pos.gx<0 || pos.gy>=GY || pos.gy<0 || pos.gtheta>=Theta || pos.gtheta<0)
		return true;

	// cout<<"Out of Bounds"<<endl;

	//first use a bounding box around car to check for collision in O(1) time
	int max_x, min_x, max_y, min_y;
	max_x=pos.x+BOT_L*abs(cos(pos.theta*2*PI/Theta))/2+BOT_W*abs(sin(pos.theta*2*PI/Theta))/2+1;
	min_x=pos.x-BOT_L*abs(cos(pos.theta*2*PI/Theta))/2-BOT_W*abs(sin(pos.theta*2*PI/Theta))/2-1;

	max_y=pos.y+BOT_L*abs(sin(pos.theta*2*PI/Theta))/2+BOT_W*abs(cos(pos.theta*2*PI/Theta))/2+1;
	min_y=pos.y-BOT_L*abs(sin(pos.theta*2*PI/Theta))/2-BOT_W*abs(cos(pos.theta*2*PI/Theta))/2-1;

	if(max_x>=MAPX || min_x<0 || max_y>=MAPY || min_y<0)
		return true;

	if(acc_obs_map[max_x][max_y]+acc_obs_map[min_x][min_y]==acc_obs_map[max_x][min_y]+acc_obs_map[min_x][max_y]){
		return false;
	}

	// cout<<"Obstacle present inside box"<<endl;

	//brute force check through the car

	for(float i=-BOT_L/2.0;i<=BOT_L/2.0+0.001;i+=1)
		for(float j=-BOT_W/2.0;j<=BOT_W/2.0+0.001;j+=1)
		{
			int s=pos.x+i*cos(pos.theta*2.0*PI/Theta)+j*sin(pos.theta*2.0*PI/Theta)+0.001;
			int t=pos.y+i*sin(pos.theta*2.0*PI/Theta)+j*cos(pos.theta*2.0*PI/Theta)+0.001;

     		if(obs_map[s][t] || obs_map[s+1][t] || obs_map[s][t+1] || obs_map[s+1][t+1] || obs_map[s-1][t-1] || obs_map[s-1][t] || obs_map[s][t-1] || obs_map[s-1][t+1] || obs_map[s+1][t-1]){
				return true;
			}
		}
	return false;
}

bool Map::is_boundary_obstacle(int i, int j)
{
        for(int k=i-1;k<=i+1;k++)
        for(int l=j-1;l<=j+1;l++)
        {
  	  if(!(k>=0&&k<MAPX&&l>=0&&l<MAPY))
  	    continue;
          if(obs_map[k][l]==0)
          {
            return true;
          }
        }

        return false;
}

void Map::find_near_obs()
{
        node node_p,node_c;

        nearest_obstacle=new int*[MAPX];

        for(int i=0;i<MAPX;i++)
        {
	  nearest_obstacle[i]=new int[MAPY];

	  for(int j=0;j<MAPY;j++)
             nearest_obstacle[i][j]=0;
        }

        queue<node> q;

        for(int i=0;i<MAPX;i++)
        {
          for(int j=0;j<MAPY;j++)
          {
            if(obs_map[i][j]==1)
            {
              if(!is_boundary_obstacle(i,j))
              {
                nearest_obstacle[i][j]=-1;
              }
              else
              {
                node_p.x=i;
                node_p.y=j;
                node_p.nearest_obstacle=1;
                q.push(node_p);
                nearest_obstacle[i][j]=1;
              }
            }
          }
       }

       while(!q.empty())
       {
         node_p=q.front();
         q.pop();

         for(int i=node_p.x-1;i<=node_p.x+1;i++)
           for(int j=node_p.y-1;j<=node_p.y+1;j++)
           {
             if(nearest_obstacle[i][j]==0&&i>=0&&i<MAPX&&j>=0&&j<MAPY&&obs_map[i][j]==0)
             {
               node_c.x=i;
               node_c.y=j;
               node_c.nearest_obstacle=node_p.nearest_obstacle+1;

               nearest_obstacle[i][j]=node_c.nearest_obstacle;
               q.push(node_c);
             }
           }
       }

       obs_dist_max=node_p.nearest_obstacle;

}

int Map::nearest_obstacle_distance(State pos)
{
      return nearest_obstacle[(int)pos.x][(int)pos.y];
}
