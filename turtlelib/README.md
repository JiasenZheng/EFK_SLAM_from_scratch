# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Perform forward and inverse kinematcs for turtlebot

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   
      * Functions
      * Member functions of a class
      * Code directly in main function

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
      * Functions<br>
      PROS:
         * For frequently used codes, functions can avoid typing the same piece of code multiple times
         * Functions divide the program in logical blocks, thus improving readability
         * Functions can be tested easily <br>
      CONS:
         * Functions are not suitable for codes that only need to be used once
         * Functions decrease execution speed <br>

      * Member functions of a class<br>
      PROS:
         * Methods can directly access the representation of a class
         * Reduces the number of functions that need to be modified after a change in representation<br>
      CONS:
         * Don't need to use member function if not accessing private members
      * Code directly in main function<br>
      PROS:
         * If the functionality only need to be used once, it uses the least amount of code
         * It increases execution speed <br>
      CONS:
         * It decreases readability
         * More codes if the functionality is used multiple times
      

   - Which of the methods would you implement and why? <br><br>
   Member function in a class is the best implementation of this functionality since the function will be used frequently, and the code can then directly access the class's private members.

* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-class) C.4: Make a function a member only if it needs direct access to the representation of a class

2. What is the difference between a class and a struct in C++?<br><br>
Members of a class are private by default while members of a struct is public by default. 


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?<br><br>
Transform2D needs to use private variables while Vector2D doesn't need.
The Vector2D is a struct because every member of the Vector2D can vary independently while Transform2D couldn't. For example, in Transform2D, sintheta depends on theta. Also, Transform2D needs some hidden member variables, so they could not be modified easily. 
* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-org) C.2: Use class if the class has an invariant; use struct if the data members can vary independently
* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-org) C.8: Use class rather than struct if any member is non-public 
4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?<br><br>
The constructor's use of the `explicit` prefix prevents `implicit` conversions. As a result, it is necessary to call the constructor explicitly with a defined format. The reason to do this is is to avoid accidental construction that might hide bugs.<br>
* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-explicit) C.46: By default, declare single-argument constructors explicit

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
By default, the member function should be declared constant so that Transform2D::inv() is constant. However, for Transform2D::operator*=(), the lhs of the operator will have to change after the operation with rhs, so that it is not declared constant. 

* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability) Con.2: By default, make member functions const

# Sample Run of frame_main
```
g++ -Wall -Wextra -g -std=c++17 -o frame_main frame_main.cpp rigid2d.cpp
./frame_main
```