//
// Created by yash on 10/20/19.
//

#ifndef SRC_CSV_READER_H
#define SRC_CSV_READER_H

/// A Class to read csv data files
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    explicit CSVReader(std::string filename, std::string delm = ",") :
            fileName(std::move(filename)), delimeter(std::move(delm))
    { }

    ///
    /// Function to fetch data from a CSV File
    std::vector<std::vector<double>> getData();
};

#endif //SRC_CSV_READER_H
