#include "../include/Compare.hpp"

State Compare::target;
int** Compare::obs_map;
float** Compare::shortest_2d;
int** Compare::grid_obs_map;

bool Compare::operator() (const State s1, const State s2){
	//to do: replace by heuristic+cost comparison
	return s1.cost3d+holonomic_with_obs(s1)+0.1*non_holonomic_without_obs(s1)>s2.cost3d+holonomic_with_obs(s2)+0.1*non_holonomic_without_obs(s2);
}

typedef bool (*compare2dSignature)(State, State);
bool compare2d(State a, State b)
{
   //return a.cost2d>b.cost2d;	//simple dijkstra
   return a.cost2d+abs(Compare::target.dx-a.dx)+abs(Compare::target.dy-a.dy)>b.cost2d+abs(Compare::target.dx-b.dx)+abs(Compare::target.dy-b.dy);
}

//currently uses dijkstra's algorithm in x-y space
float Compare::holonomic_with_obs(State src){

	return Compare::shortest_2d[(int)src.x*DX/MAPX][(int)src.y*DY/MAPY];
}

void Compare::runDijkstra(){

	State src=Compare::target;
	src.dx=src.gx*DX/GX;
	src.dy=src.gy*DY/GY;
	priority_queue<State, vector<State>, compare2dSignature> frontier(&compare2d);
	int vis[DX][DY];

	float** cost=new float*[DX];
	for(int i=0;i<DX;i++)
	{
		cost[i]=new float[DY];
		for(int j=0;j<DY;j++)
			cost[i][j]=0;
	}

	memset(vis, 0, sizeof(int) * DX * DY);

	for(int i=0;i<DX;i++)
		for(int j=0;j<DY;j++)
			cost[i][j]=10000;
	cost[src.dx][src.dy]=0;

	frontier.push(src);
	while(!frontier.empty()){
		State current=frontier.top();
		frontier.pop();

		int x=current.dx;
		int y=current.dy;

		if(vis[x][y])
			continue;

		vis[x][y]=1;

		for(int i=-1;i<=1;i++)
			for(int j=-1;j<=1;j++)
			{
				if(x+i<0 || x+i>=DX || y+j<0 || y+j>=DY)
					continue;
				if((i==0 && j==0) || Compare::grid_obs_map[x+i][y+j]!=0)
					continue;

				if(cost[x+i][y+j]>cost[x][y]+sqrt(i*i+j*j))
				{
					cost[x+i][y+j]=cost[x][y]+sqrt(i*i+j*j);
					State tempstate;
					tempstate.dx=current.dx+i;
					tempstate.dy=current.dy+j;
					tempstate.cost2d=cost[x+i][y+j];
					frontier.push(tempstate);
				}
			}
	}
	Compare::shortest_2d=cost;

	Mat dist(240, 240, CV_8UC3, Scalar(255, 255, 255));
	for(int i=0;i<240;i++)
		for(int j=0;j<240;j++)
		{
			dist.at<Vec3b>(j,i)={255-0.6*shortest_2d[i][j], 200-0.6*shortest_2d[i][j], 200-0.6*shortest_2d[i][j]};
		}
	resize(dist, dist, Size(400, 400));
	//uncomment to check if dijkstra ran properly
	//imshow("dist", dist);
	//waitKey(0);
}

struct pt{
	float x;
	float y;
};

int quad(struct pt a, struct pt b){
  if(b.x>a.x && b.y>a.y) return 1;
  else if(b.x<a.x && b.y>a.y) return 2;
  else if(b.x<a.x && b.y<a.y) return 3;
  else if(b.x>a.x && b.y<a.y) return 4;
}

float Compare::non_holonomic_without_obs(State src){

	//return abs(Compare::target.x-src.x)+abs(Compare::target.y-src.y)+abs(Compare::target.theta-src.theta);
	float rmin=(BOT_L/tan((BOT_M_ALPHA)*PI/180));

	struct pt CS, ACS, CE, ACE;

	CS.x= src.dx+(rmin)*sin((src.theta)*PI/180); //Clockwise-Start
	CS.y= src.dy-(rmin)*cos((src.theta)*PI/180);

	ACS.x= src.dx-(rmin)*sin((src.theta)*PI/180); //Anti-Clockwise-Start
	ACS.y= src.dy+(rmin)*cos((src.theta)*PI/180);

	CE.x= Compare::target.dx+(rmin)*sin((Compare::target.theta)*PI/180); //Clockwise-End
	CE.y= Compare::target.dy-(rmin)*cos((Compare::target.theta)*PI/180);

	ACE.x= Compare::target.dx-(rmin)*sin((Compare::target.theta)*PI/180); //Anti-Clockwise-End
	ACE.y= Compare::target.dy+(rmin)*cos((Compare::target.theta)*PI/180);

	float lcc=sqrt((CS.x-CE.x)*(CS.x-CE.x)+(CS.y-CE.y)*(CS.y-CE.y)); //clockwise-clockwise length
	float laa=sqrt((ACS.x-ACE.x)*(ACS.x-ACE.x)+(ACS.y-ACE.y)*(ACS.y-ACE.y));
	float lca=sqrt((CS.x-ACE.x)*(CS.x-ACE.x)+(CS.y-ACE.y)*(CS.y-ACE.y));
	float lac=sqrt((ACS.x-CE.x)*(ACS.x-CE.x)+(ACS.y-CE.y)*(ACS.y-CE.y));

	struct pt A,B;
//Code for CCC
	float pathCCC=INT_MAX; //As CCC is not always possible
	if(!(lcc>4*rmin && laa>4*rmin)){
		int d;
		d=min(lcc,laa);
		int flag; //Flag=1 for clockwise being optimal and 0 for anticlockwise
		if(d==lcc){
			flag=1;
			A=CS;
			B=CE;
		}
		else{
			flag=0;
			A=ACS;
			B=ACE;
		}

		double n;
		n=((B.y-A.y)/(B.x-A.x));
		float t=atan(n);
		if(quad(A,B)==2 || quad(A,B)==3) t=PI+t;
  		else if(quad(A,B)==4) t=2*PI+t;
		n=d/(4*rmin);
		float p=acos(n);

		float ccc[4];

		if(flag==0){ //anti-clockwise being optimal
			ccc[0]=(PI)/2 -src.theta*PI/180+t-p; //Angle traversed in ACS for Path through Lower intermediate circle
			ccc[1]=(PI)/2 +Compare::target.theta*PI/180-t-p; //Angle traversed in ACE for Path through Lower intermediate circle
			ccc[2]=(PI)/2 -src.theta*PI/180+t+p; //Angle traversed in ACS for Path through Upper intermediate circle
			ccc[3]=(PI)/2 +Compare::target.theta*PI/180-t+p; //Angle traversed in ACE for Path through Upper intermediate circle
		}
		else{ //clockwise being optimal
			ccc[0]=(PI)/2 +src.theta*PI/180-t+p;	//Angle traversed in CS for Path through Lower intermediate circle
			ccc[1]=(PI)/2 -Compare::target.theta*PI/180+t+p;	//Angle traversed in CE for Path through Lower intermediate circle
			ccc[2]=(PI)/2 +src.theta*PI/180-t-p;	//Angle traversed in CS for Path through Upper intermediate circle
			ccc[3]=(PI)/2 -Compare::target.theta*PI/180+t-p;	//Angle traversed in CE for Path through Upper intermediate circle
		}

		int i;
		for(i=0;i<4;i++){
			if(ccc[i]<0)	ccc[i]=ccc[i]+2*PI;
			else if (ccc[i]>=2*(PI))	ccc[i]=ccc[i]-2*PI;
		}

		float L,U;
		if(flag==0){ //anticlockwise
			L=ccc[0]+ccc[1]+PI-2*p;
			U=ccc[2]+ccc[3]+PI+2*p;
		}
		else { //clockwise
			L=ccc[0]+ccc[1]+PI+2*p;
			U=ccc[2]+ccc[3]+PI-2*p;
		}

		pathCCC=(rmin)*(min(L,U));
	}

	//Code for CSC
	float csc[8];
	float dis[4]; //Array to store final distance traversed in the 4 possible cases
	float theta;  //Angle made by line joining centres of two circular paths with x-axis
	double N;
	float ltct_ac=INT_MAX, ltct_ca=INT_MAX;

	A=CS;  B=CE; //For Direct Common Tangent -clockwise to clockwise
	N=((B.y-A.y)/(B.x-A.x));
	theta=atan(N);
	if(quad(A,B)==2 || quad(A,B)==3) theta=PI+theta;
  	else if(quad(A,B)==4) theta=2*PI+theta;
	csc[0]=src.theta*PI/180-theta;
	csc[1]=theta-Compare::target.theta*PI/180;

	A=ACS;  B=ACE; //For Direct Common Tangent -anticlockwise to anticlockwise
	N=((B.y-A.y)/(B.x-A.x));
	theta=atan(N);
	if(quad(A,B)==2 || quad(A,B)==3) theta=PI+theta;
  	else if(quad(A,B)==4) theta=2*PI+theta;
	csc[2]=src.theta*PI/180-theta;
	csc[3]=theta-Compare::target.theta*PI/180;

	if(lac>2*(rmin)){//For Transverse Common Tangent-anticlockwise to clockwise
		A=ACS;  B=CE;
		N=((B.y-A.y)/(B.x-A.x));
		theta=atan(N);
		if(quad(A,B)==2 || quad(A,B)==3) theta=PI+theta;
  		else if(quad(A,B)==4) theta=2*PI+theta;
		float phi_ac=acos(2*rmin/lac);
		csc[4]=PI/2-src.theta*PI/180+theta-phi_ac;
		csc[5]=PI/2-Compare::target.theta*PI/180+theta-phi_ac;
		ltct_ac=sqrt(lac*lac-4*rmin*rmin); //length of TCT-anticlockwise to clockwise
	}
	else{
		csc[4]=INT_MAX;
		csc[5]=INT_MAX;
	}

	if(lca>2*(rmin)){//For Transverse Common Tangent -clockwise to anticlockwise
		A=CS;  B=ACE;
		N=((B.y-A.y)/(B.x-A.x));
		theta=atan(N);
  		if(quad(A,B)==2 || quad(A,B)==3) theta=PI+theta;
  		else if(quad(A,B)==4) theta=2*PI+theta;
		float phi_ca=acos(2*rmin/lca);
		csc[6]=PI/2+src.theta*PI/180-theta-phi_ca;
		csc[7]=PI/2+Compare::target.theta*PI/180-theta-phi_ca;
		ltct_ca=sqrt(lca*lca-4*rmin*rmin); //lenth of TCT-clockwise to anticlockwise
	}
	else{
		csc[6]=INT_MAX;
		csc[7]=INT_MAX;
	}

	int i;
	for(i=0;i<8;i++){
		if(csc[i]>-0.0001 && csc[i]<0.0001) csc[i]=0;
		if(csc[i]<0)	csc[i]=csc[i]+2*PI;
		else if (csc[i]>=2*(PI))	csc[i]=csc[i]-2*PI;
	}

	dis[0]=(rmin)*(csc[0]+csc[1])+lcc; //For DCT-clockwise
	dis[1]=(rmin)*(csc[2]+csc[3])+lcc; //For DCT-anticlockwise
	dis[2]=(rmin)*(csc[4]+csc[5])+ltct_ac; //For TCT-anticlockwise to clockwise
	dis[3]=(rmin)*(csc[6]+csc[7])+ltct_ca; //For TCT-clockwise to anticlockwise

	float pathCSC=INT_MAX; //To find min of all possible CSC paths
	for(i=0;i<4;i++){
		if(dis[i]<pathCSC){
			pathCSC=dis[i];
		}
	}

	if(pathCCC>pathCSC) return pathCSC;
	return pathCCC;
}
