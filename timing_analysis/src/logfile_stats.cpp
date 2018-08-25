#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <algorithm>
#include <numeric>
using namespace std;


vector<double> compStats (vector<double> v) {
    double sum = accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    vector<double> diff(v.size());
    transform(v.begin(), v.end(), diff.begin(),
               [mean](double x) { return x - mean; });
    double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = sqrt(sq_sum / v.size());

    vector<double> results;
    results.push_back(mean);
    results.push_back(stdev);
    return results;
}


void logStats (string filename, int number_of_measurements) {

    ifstream inFile;
    vector<double> vect_proc;
    vector<double> vect_overall;

    string dstring;
    string input;

    float d = 0;
    double min_p = 100;
    double min_o = 100;
    double max_p = 0;
    double max_o = 0;

    int min_idx = 0;
    int max_idx = 0;
    int counter = 1;

    inFile.open(filename);

    if (!inFile) {
        cerr << "Unable to open file " << filename << endl;
        return;   // call system to stop
    }

    //skip first line
    getline(inFile, dstring);
    // skip first number (time)
    getline(inFile, dstring, ',');
    while (getline(inFile, dstring, ',') &&
            vect_overall.size() < number_of_measurements) {

        istringstream dss(dstring);
        dss >> d;
        if (counter%3 == 0) {
            if (d < min_o) {
                min_o = d;
                min_idx = vect_overall.size();
            }
            else if (d > max_o) {
                max_o = d;
                max_idx = vect_overall.size();
            }
            vect_overall.push_back(d);
            }
        else if ((counter+1)%3 == 0) {
             if (d < min_p)
                min_p = d;
             else if (d > max_p)
                max_p = d;
             vect_proc.push_back(d);
        }
        counter++;
    }

    inFile.close();

    cout << endl;
    cout << "########## LOGGING RESULTS ##########";
    cout << endl << endl;
    cout << "File: " << filename << endl;
    cout << "Number of Measurements: " << vect_proc.size();
    cout << endl << endl;

    cout << "##############################";
    cout << endl << endl;

    cout << "BARE PROCESSING TIME:"; 
    cout << endl << endl;
    cout << "Mean = " << compStats(vect_proc)[0] << endl;
    cout << "Min = " << min_p << endl;
    cout << "Max = " << max_p << endl;
    cout << "Stdev = " << compStats(vect_proc)[1] << endl;
    cout << endl << endl;

    cout << "##############################";
    cout << endl << endl;

    cout << "TOTAL ELAPSED TIME:";
    cout << endl << endl;
    cout << "Mean = " << compStats(vect_overall)[0] << endl;
    cout << "Min = " << min_o;
    cout << "; Corresponding processing time = " << vect_proc[min_idx];
    cout << endl;
    cout << "Max = " << max_o;
    cout << "; Corresponding processing time = " << vect_proc[max_idx];
    cout << endl;
    cout << "Stdev " << compStats(vect_overall)[1]; 
    cout << endl << endl;
}

int main(int argc, char* argv[]) {

    string logfile;

    cout << "Please enter a log-filename that you want to analyze: \n";
    cin >> logfile;

    logfile = "/home/patrick/LOGFILES/" + logfile;            
    logStats(logfile, 20000);

    return 0;
}
