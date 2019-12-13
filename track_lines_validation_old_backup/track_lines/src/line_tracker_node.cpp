


#include "lane_tracker.h"


/*! \mainpage Autopilot Introduction
 *
 * \section None Programing Style Guide
 *
 * After Google C++ Style Guide -> https://google.github.io/styleguide/cppguide.html
 *
 * \subsection None The #define Guard
 * All header files should have #define guards to prevent multiple inclusion.
 *
 * \subsection None1 Names and Order of Includes
 * Use standard order for readability and to avoid hidden dependencies: Related header, C library, C++ library, other libraries' .h, your project's .h.
 *
 * \subsection None2 General Naming Rules
 * Names should be descriptive; avoid abbreviation.
 * Give as descriptive a name as possible, within reason. Do not worry about saving horizontal space as it is far more important to make your code immediately understandable by a new reader.
 *
 * \subsection None3 Declaration Order
 * A class definition should usually start with a public: section, followed by protected:, then private:. Omit sections that would be empty.
 *
 * \subsection None4 Variable Names
 * The names of variables (including function parameters) and data members are all lowercase, with underscores between words. Data members of classes (but not structs) additionally have trailing underscores.
 *
 * \subsection None5 Class Data Members
 * Data members of classes, both static and non-static, are named like ordinary nonmember variables, but with a trailing underscore.
 *
 * \subsection None6 Constant_Names
 * Variables declared constexpr or const, and whose value is fixed for the duration of the program, are named with a leading "k" followed by mixed case.
 * Underscores can be used as separators in the rare cases where capitalization cannot be used for separation.
 *
 * \subsection None7 Function Names
 * Regular functions have mixed case; accessors and mutators may be named like variables.
 * Ordinarily, functions should start with a capital letter and have a capital letter for each new word.
 *
 * \subsection None8 Comment Style
 * Use either the // or /* syntax, as long as you are consistent.
 *
 * \subsection None9 TODO Comments
 * Use TODO comments for code that is temporary, a short-term solution, or good-enough but not perfect.
 *
 * \subsection None10 Line Length
 * Each line of text in your code should be at most 80 characters long.
 *
 * \subsection None11 Macro Names
 * If they are absolutely needed, then they should be named with all capitals and underscores.

*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_tracker_node");
    ros::NodeHandle node_handle;

    LaneTracker tracker(&node_handle);

    ros::spin();
    ros::shutdown();

    return 0;
}
