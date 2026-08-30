#include "thread_terrain_match.h"

ThreadTerrainMatch::ThreadTerrainMatch(QObject *parent)
{
}

ThreadTerrainMatch::~ThreadTerrainMatch()
{
    stop_match();
}

void ThreadTerrainMatch::match()
{
    stop_thread = false;
    sem_init(&tercom_recv_data_flag, 0, 0);
    sem_init(&tercom_flag, 0, 0);

    vector<double> data_temp;

    ifstream inputfile("integrated_navigation/initial_posi.txt");
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

    thread1 = std::thread(recv_thread); 
    usleep(100);
    thread2 = std::thread(send_thread);
    usleep(100);
    thread5 = std::thread(SITAN_thread);
    usleep(100);
    thread3 = std::thread(fusion_thread);
    usleep(100);
    thread4 = std::thread(TERCOM_thread);
    usleep(100);

    // std::thread t1(recv_thread);
    // usleep(100);
    // std::thread t2(send_thread);
    // usleep(100);
    // std::thread t5(SITAN_thread);
    // usleep(100);
    // std::thread t3(fusion_thread);
    // usleep(100);
    // std::thread t4(TERCOM_thread);

    // t1.join();
    // t2.join();
    // t5.join();
    // t3.join();
    // t4.join();

    // sem_destroy(&tercom_recv_data_flag);
    // sem_destroy(&tercom_flag);

}

void ThreadTerrainMatch::stop_match()
{
    
    stop_thread = true;
    usleep(100000);
    
    if (thread1.joinable()) 
    {
        thread1.join();
    }
     
    if (thread2.joinable()) 
    {
        thread2.join();
    }
     
    if (thread5.joinable()) 
    {
        thread5.join();
    }
     
    if (thread3.joinable()) 
    {
        thread3.join();
    }
    
    if (thread4.joinable()) 
    {
        thread4.join();
    }
     

    sem_destroy(&tercom_recv_data_flag);
    sem_destroy(&tercom_flag);

    emit sig_match_finished();
}