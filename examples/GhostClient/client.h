
#include <vector>

using namespace std;

struct AnalyticsData {
   int islandId; 
   int numBodies;
   int numIterationsUsed;
   double remainingResidual;

};

int connect();
vector<int> loadSDF(const char* fileName);
vector<AnalyticsData> stepSimulation();