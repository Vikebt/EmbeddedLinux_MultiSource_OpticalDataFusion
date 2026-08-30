#include "data_struct.h"
#include "muti_thread.h"
#include <thread>
using namespace std;

stack<data_recv> data_recv_pool;
stack<data_recv> data_recv_pool_sitan;
stack<data_send> data_send_pool;
stack<TERCOM_recv> TERCOM_recv_pool;
stack<TERCOM_send> TERCOM_send_pool;
stack<SITAN_send> SITAN_send_pool;
vector<double> initial_posi(3, 1.0);
vector<double> initial_atti(3, 1.0);
vector<double> initial_velo(3, 1.0);
mutex mymutex;
sem_t tercom_recv_data_flag;
sem_t tercom_flag;
TERCOM t1;
TERCOM t2;
// SITAN s1;

int main(int argc, const char **argv)
{
    sem_init(&tercom_recv_data_flag, 0, 0);
    sem_init(&tercom_flag, 0, 0);

    vector<double> data_temp;
    ifstream inputfile("initial_posi.txt");
    if (!inputfile.is_open())
    {
        cout << "file open error!" << endl;
    }
    string line;
    while (getline(inputfile, line))
    {
        double value;
        if (istringstream(line) >> value)
        {
            data_temp.push_back(value);
        }
    }
    inputfile.close();

    initial_posi[0] = data_temp[0];
    initial_posi[1] = data_temp[1];
    initial_posi[2] = data_temp[2];

    initial_atti[0] = data_temp[3];
    initial_atti[1] = data_temp[4];
    initial_atti[2] = data_temp[5];

    initial_velo[0] = data_temp[6];
    initial_velo[1] = data_temp[7];
    initial_velo[2] = data_temp[8];

    thread t1(recv_thread);
    usleep(100);
    thread t2(send_thread);
    usleep(100);
    thread t5(SITAN_thread);
    usleep(100);
    thread t3(fusion_thread);
    usleep(100);
    thread t4(TERCOM_thread);


    t1.join();
    t2.join();
    t5.join();
    t3.join();
    t4.join();

    sem_destroy(&tercom_recv_data_flag);
    sem_destroy(&tercom_flag);

    return 0;
}