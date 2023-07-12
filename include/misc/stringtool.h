#ifndef STRING_TOOL_H
#define STRING_TOOL_H

#include <string>
#include <vector>

class StringToolException : public std::exception
{
public:
    StringToolException(char* whatMsg)
        : m_whatMsg(whatMsg) {}

    inline const char* what() const override {
        return m_whatMsg;
    }

private:
    char* m_whatMsg;
};

class StringTool
{
public:
    inline static double castStringToDouble(const std::string& src) {
        try {
            return stod(src);
        } catch (std::invalid_argument &) {
            throw StringToolException("String is not a valid double!");
        } catch (std::out_of_range &) {
            throw StringToolException("String value does not fit in a double!");
        }
    }

    inline static float castStringToFloat(const std::string& src) {
        try {
            return stof(src);
        }
        catch (std::invalid_argument&) {
            throw StringToolException("String is not a valid float!");
        }
        catch (std::out_of_range&) {
            throw StringToolException("String value does not fit in a float!");
        }
    }
 
    inline static void trim(std::string& outSrc) {
        size_t start = outSrc.find_first_not_of(" \t\n\r\f\v");
        if (start == std::string::npos) {
            outSrc = "";
            return;
        }

        size_t end = outSrc.find_last_not_of(" \t\n\r\f\v");
        outSrc = outSrc.substr(start, end - start + 1);
        return;
    }


    inline static std::vector<string> split(const std::string& src, std::string&& delimiter) {
        return split(src, delimiter);
    }

    inline static std::vector<string> split(const std::string& src, const std::string& delimiter) {
        size_t start = 0; 
        size_t end = 0;
        std::vector<std::string> result;
        while ((end = src.find(delimiter, start)) != string::npos) {
            result.push_back(src.substr(start, end - start));
            start = end + 1;
        } 
        result.push_back(src.substr(start, end - start));
        return result;
    }
};

#endif STRING_TOOL_H