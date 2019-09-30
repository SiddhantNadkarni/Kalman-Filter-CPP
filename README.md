# Kalman-Filter-CPP
Simple Kalman Filter Implementation to track the position of an object of unknown dynamics and moves in one dimension

## Getting Started

 We want to track the position of an object of unknown dynamics. We assume that
the object moves in one dimension. A common model for unkown dynamics is the
constant jerk model, that is assume that the acceleration is linear. The constant jerk
model can be represented as follows: x(t) = [pt, vt, at, jt].tranpose(), with the elements being
position, velocity, acceleration and jerk.

### Prerequisites

C++ [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) Library

### Installing


```
git clone https://github.com/SiddhantNadkarni/Kalman-Filter-CPP.git
cd Kalman-Filter-CPP
mkdir build && cd build
cmake ..
./KalmanFilter
```

## Authors

* **Siddhant Nadkarni** 


## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/SiddhantNadkarni/Kalman-Filter-CPP/blob/master/LICENSE) file for details

## Acknowledgments

* [Probabilities Robotics Textbook](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf) 
* Parts of implementation structure referred from [PrieureDeSion's Implementation](https://github.com/PrieureDeSion/kalmanfilter-cpp)
* etc

