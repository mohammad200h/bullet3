
#include <stdio.h>
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#ifdef BT_ENABLE_ENET
#include "../SharedMemory/PhysicsClientUDP_C_API.h"
#endif  //BT_ENABLE_ENET
#define PYBULLET_PI (3.1415926535897932384626433832795029)

#ifdef BT_ENABLE_DART
#include "../SharedMemory/dart/DARTPhysicsC_API.h"
#endif

#ifdef BT_ENABLE_PHYSX
#include "../SharedMemory/physx/PhysXC_API.h"
#endif

#ifdef BT_ENABLE_MUJOCO
#include "../SharedMemory/mujoco/MuJoCoPhysicsC_API.h"
#endif

#ifdef BT_ENABLE_GRPC
#include "../SharedMemory/PhysicsClientGRPC_C_API.h"
#endif
#ifdef BT_ENABLE_CLSOCKET
#include "../SharedMemory/PhysicsClientTCP_C_API.h"
#endif  //BT_ENABLE_CLSOCKET

#if defined(__APPLE__) && (!defined(B3_NO_PYTHON_FRAMEWORK))
#include <Python/Python.h>
#else
#ifdef _WIN32
	#ifdef _DEBUG
		#define BT_REMOVED_DEBUG
		//always use the release build of Python
		#undef _DEBUG
	#endif //_DEBUG
#endif

#endif
#ifdef BT_REMOVED_DEBUG
	#define _DEBUG
#endif
#include "../Importers/ImportURDFDemo/urdfStringSplit.h"

#ifdef B3_DUMP_PYTHON_VERSION
#define B3_VALUE_TO_STRING(x) #x
#define B3_VALUE(x) B3_VALUE_TO_STRING(x)
#define B3_VAR_NAME_VALUE(var) #var "=" B3_VALUE(var)
#pragma message(B3_VAR_NAME_VALUE(PY_MAJOR_VERSION))
#pragma message(B3_VAR_NAME_VALUE(PY_MINOR_VERSION))
#endif

#ifdef PYBULLET_USE_NUMPY
#include <numpy/arrayobject.h>
#endif

#if PY_MAJOR_VERSION >= 3
#define PyInt_FromLong PyLong_FromLong
#define PyString_FromString PyBytes_FromString
#endif

#include "client.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

#define B3_MAX_NUM_END_EFFECTORS 128
#define MAX_PHYSICS_CLIENTS 1024
static b3PhysicsClientHandle sPhysicsClients1[MAX_PHYSICS_CLIENTS] = {0};
static int sPhysicsClientsGUI[MAX_PHYSICS_CLIENTS] = {0};
static int sNumPhysicsClients = 0;
using namespace std;


b3PhysicsClientHandle getPhysicsClient(int physicsClientId)
{
	b3PhysicsClientHandle sm;
	if ((physicsClientId < 0) || (physicsClientId >= MAX_PHYSICS_CLIENTS) || (0 == sPhysicsClients1[physicsClientId]))
	{
		return 0;
	}
	sm = sPhysicsClients1[physicsClientId];
	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			return sm;
		}
		//broken connection?
		b3DisconnectSharedMemory(sm);
		sPhysicsClients1[physicsClientId] = 0;
		sPhysicsClientsGUI[physicsClientId] = 0;

		sNumPhysicsClients--;
	}
	return 0;
}



//pybullet_connectPhysicsServer
int connect(){
	int freeIndex = -1;
	int method = eCONNECT_GUI;
	int i;
	char* options = 0;
	// printf("ASSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
	b3PhysicsClientHandle sm = 0;

	if (sNumPhysicsClients >= MAX_PHYSICS_CLIENTS)
	{
		// PyErr_SetString(SpamError,
		// 				"Exceeding maximum number of physics connections.");
		return NULL;
	}

	{
		int key = SHARED_MEMORY_KEY;
		int udpPort = 1234;
		int tcpPort = 6667;
		int grpcPort = -1;
		int argc = 0;
		char** argv = 0;

		const char* hostName = "localhost";

		static char* kwlist1[] = {"method", "key", "options", NULL};
		static char* kwlist2[] = {"method", "hostName", "port", "options", NULL};



		//Only one local in-process GUI connection allowed.
		if (method == eCONNECT_GUI)
		{
			int i;
			for (i = 0; i < MAX_PHYSICS_CLIENTS; i++)
			{
				if ((sPhysicsClientsGUI[i] == eCONNECT_GUI) || (sPhysicsClientsGUI[i] == eCONNECT_GUI_SERVER))
				{
					// PyErr_SetString(SpamError,
									// "Only one local in-process GUI/GUI_SERVER connection allowed. Use DIRECT connection mode or start a separate GUI physics server (ExampleBrowser, App_SharedMemoryPhysics_GUI, App_SharedMemoryPhysics_VR) and connect over SHARED_MEMORY, UDP or TCP instead.");
					return NULL;
				}
			}
		}

		if (options)
		{
			int i;
			argv = urdfStrSplit(options, " ");
			argc = urdfStrArrayLen(argv);
			for (i = 0; i < argc; i++)
			{
				printf("argv[%d]=%s\n", i, argv[i]);
			}
		}
		switch (method)
		{
	
			case eCONNECT_GUI:
			{
#ifdef __APPLE__
				sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
				sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
				break;
			}
			case eCONNECT_GUI_MAIN_THREAD:
			{
				sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
				break;
			}
			case eCONNECT_GUI_SERVER:
			{
#ifdef __APPLE__
				sm = b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(argc, argv);
#else
				sm = b3CreateInProcessPhysicsServerAndConnectSharedMemory(argc, argv);
#endif
				break;
			}
			case eCONNECT_GRAPHICS_SERVER_MAIN_THREAD:
			{
				sm = b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(tcpPort);
				break;
			}
			case eCONNECT_GRAPHICS_SERVER:
			{
#ifdef __APPLE__
				sm = b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(tcpPort);
#else
				sm = b3CreateInProcessGraphicsServerAndConnectSharedMemory(tcpPort);
#endif
				break;
			}
			case eCONNECT_SHARED_MEMORY_SERVER:
			{
				sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3(0, key);
				break;
			}

			case eCONNECT_SHARED_MEMORY_GUI:
			{
				sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect4(0, key);
				break;
			}

			case eCONNECT_GRAPHICS_SERVER_TCP:
			{
#ifdef BT_ENABLE_CLSOCKET
				sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnectTCP(hostName, tcpPort);
#else
				// PyErr_SetString(SpamError, "TCP is not enabled in this pybullet build");
				return NULL;
#endif//BT_ENABLE_CLSOCKET
				break;
			}

			
			case eCONNECT_DIRECT:
			{
				sm = b3ConnectPhysicsDirect();
				break;
			}
#ifdef BT_ENABLE_DART
			case eCONNECT_DART:
			{
				sm = b3ConnectPhysicsDART();
				break;
			}
#endif
#ifdef BT_ENABLE_PHYSX
			case eCONNECT_PHYSX:
			{
				sm = b3ConnectPhysX(argc, argv);
				break;
			}
#endif

#ifdef BT_ENABLE_MUJOCO
			case eCONNECT_MUJOCO:
			{
				sm = b3ConnectPhysicsMuJoCo();
				break;
			}
#endif
			case eCONNECT_GRPC:
			{
#ifdef BT_ENABLE_GRPC
				sm = b3ConnectPhysicsGRPC(hostName, grpcPort);
#else
				// PyErr_SetString(SpamError, "GRPC is not enabled in this pybullet build");
#endif
				break;
			}
			case eCONNECT_SHARED_MEMORY:
			{
				sm = b3ConnectSharedMemory(key);
				break;
			}
			case eCONNECT_UDP:
			{
#ifdef BT_ENABLE_ENET

				sm = b3ConnectPhysicsUDP(hostName, udpPort);
#else
				// PyErr_SetString(SpamError, "UDP is not enabled in this pybullet build");
				return NULL;
#endif  //BT_ENABLE_ENET

				break;
			}
			case eCONNECT_TCP:
			{
#ifdef BT_ENABLE_CLSOCKET

				sm = b3ConnectPhysicsTCP(hostName, tcpPort);
#else
				// PyErr_SetString(SpamError, "TCP is not enabled in this pybullet build");
				return NULL;
#endif  //BT_ENABLE_CLSOCKET

				break;
			}

			default:
			{
				// PyErr_SetString(SpamError, "connectPhysicsServer unexpected argument");
				return NULL;
			}
		};

		if (options)
		{
			urdfStrArrayFree(argv);
		}
	}

	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			for (i = 0; i < MAX_PHYSICS_CLIENTS; i++)
			{
				if (sPhysicsClients1[i] == 0)
				{
					freeIndex = i;
					break;
				}
			}

			if (freeIndex >= 0)
			{
				b3SharedMemoryCommandHandle command;
				b3SharedMemoryStatusHandle statusHandle;
				int statusType;

				sPhysicsClients1[freeIndex] = sm;
				sPhysicsClientsGUI[freeIndex] = method;
				sNumPhysicsClients++;

				if (method != eCONNECT_GRAPHICS_SERVER && method != eCONNECT_GRAPHICS_SERVER_MAIN_THREAD)
				{
					command = b3InitSyncBodyInfoCommand(sm);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
					statusType = b3GetStatusType(statusHandle);

					if (statusType != CMD_SYNC_BODY_INFO_COMPLETED)
					{
						printf("Connection terminated, couldn't get body info\n");
						b3DisconnectSharedMemory(sm);
						sm = 0;
						sPhysicsClients1[freeIndex] = 0;
						sPhysicsClientsGUI[freeIndex] = 0;
						sNumPhysicsClients++;
						return -1;
					}

					command = b3InitSyncUserDataCommand(sm);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
					statusType = b3GetStatusType(statusHandle);

					if (statusType != CMD_SYNC_USER_DATA_COMPLETED)
					{
						printf("Connection terminated, couldn't get user data\n");
						b3DisconnectSharedMemory(sm);
						sm = 0;
						sPhysicsClients1[freeIndex] = 0;
						sPhysicsClientsGUI[freeIndex] = 0;
						sNumPhysicsClients++;
						return -1;
					}
				}
			}
		}
		else
		{
			b3DisconnectSharedMemory(sm);
		}
	}
	return freeIndex;
}



//pybullet_loadSDF
vector<int> loadSDF(const char* fileName){
	const char* sdfFileName = "";
	int numBodies = 0;
	int i;
	int bodyIndicesOut[MAX_SDF_BODIES];
	int useMaximalCoordinates = -1;

	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle commandHandle;
	b3PhysicsClientHandle sm = 0;
	double globalScaling = -1;

	int physicsClientId = 0;
	static char* kwlist[] = {"sdfFileName", "useMaximalCoordinates", "globalScaling", "physicsClientId", NULL};

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		
		cout<<"Not connected to physics server."<<endl;
		return {};
	}

	commandHandle = b3LoadSdfCommandInit(sm, fileName);
	if (useMaximalCoordinates > 0)
	{
		b3LoadSdfCommandSetUseMultiBody(commandHandle, 0);
	}
	if (globalScaling > 0)
	{
		b3LoadSdfCommandSetUseGlobalScaling(commandHandle, globalScaling);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_SDF_LOADING_COMPLETED)
	{
		
		cout<<"Cannot load SDF file."<<endl;
		return {};
	}

	numBodies =
		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
	if (numBodies > MAX_SDF_BODIES)
	{
		char str[1024];
		sprintf(str, "SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
		
		cout<<str<<endl;
		return {};
	}

	// pylist = PyTuple_New(numBodies);
	int indexes[numBodies];


	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES)
	{
		for (i = 0; i < numBodies; i++)
		{
			// PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i]));
			indexes[i]= bodyIndicesOut[i];

		}
	}
	vector<int> vec_indexes(indexes, indexes + sizeof indexes / sizeof indexes[0]);
	
	return vec_indexes;
}



//pybullet_resetBasePositionAndOrientation
void resetBasePositionAndOrientation(int bodyUniqueId,vector<float>posObj, vector<float> ornObj){

}


//pybullet_stepSimulation
vector<AnalyticsData> stepSimulation(){
	int physicsClientId = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	b3PhysicsClientHandle sm = 0;


	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		
		cout<< "Not connected to physics server."<<endl;
		return {};
	}

	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		if (b3CanSubmitCommand(sm))
		{
			statusHandle = b3SubmitClientCommandAndWaitStatus(
				sm, b3InitStepSimulationCommand(sm));
			statusType = b3GetStatusType(statusHandle);

			if (statusType == CMD_STEP_FORWARD_SIMULATION_COMPLETED)
			{
				struct b3ForwardDynamicsAnalyticsArgs analyticsData;
				int numIslands = 0;
				int i;
				// PyObject* val = 0;
				// PyObject* pyAnalyticsData;
				
                                
				numIslands = b3GetStatusForwardDynamicsAnalyticsData(statusHandle, &analyticsData);
				vector<AnalyticsData> analyticsData_vect;
				// pyAnalyticsData = PyTuple_New(numIslands);
                                
				for (i=0;i<numIslands;i++)
				{
				
					struct AnalyticsData tmp;
					tmp.islandId  =analyticsData.m_islandData[i].m_islandId;
					tmp.numBodies  =analyticsData.m_islandData[i].m_numBodies;
					tmp.numIterationsUsed  =analyticsData.m_islandData[i].m_numIterationsUsed;
					tmp.remainingResidual  =analyticsData.m_islandData[i].m_remainingLeastSquaresResidual;

					analyticsData_vect[i]=tmp;
				
				}

				return analyticsData_vect;
			}
		}
	}


	return {};

    
}