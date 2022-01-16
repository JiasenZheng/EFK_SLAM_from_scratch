# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?<br><br>
Members of a class are private by default while members of a struct is public by default. As a result, a structure is not secure and cannot hide its implementation details from the end-user.

* Reference: https://www.geeksforgeeks.org/structure-vs-class-in-cpp/


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?<br><br>
Transform2D needs to use private variable while Vector2D deesn't need.

* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-org) C.2: Use class if the class has an invariant; use struct if the data members can vary independently
4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?<br><br>
Using `explict` prefix to the constructor prevent `implicit` conversions. As a result, it is necessary to call the constructor explicitly with defined format. The reason to do this is is to avoid accidental construction that might hide bugs.<br>
* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-explicit) C.46: By default, declare single-argument constructors explicit

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

# Sample Run of frame_main
```
<Put the output of a representative run here>
```