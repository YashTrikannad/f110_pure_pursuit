//
// Created by yash on 10/20/19.
//

#ifndef SRC_CSV_READER_CPP
#define SRC_CSV_READER_CPP

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

#include "csv_reader.h"

/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
std::vector<std::vector<double> > CSVReader::getData()
{
    std::ifstream file(fileName);

    std::vector<std::vector<double> > dataList;

    std::string line = "";
    // Iterate through each line and split the content using delimeter
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        std::vector<double> vec_double;
        for(const auto& word:vec)
        {
            vec_double.emplace_back(std::stod(word));
        }
        dataList.push_back(vec_double);
    }
    // Close the File
    file.close();

    return dataList;
}

#endif //SRC_CSV_READER_CPP
