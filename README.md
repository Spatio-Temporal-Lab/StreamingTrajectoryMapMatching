# EAR-MM: An Efficient Adaptive and Robust Algorithm for Streaming Map Matching
***
EAR-MM is an efficient, adaptive and robust Algorithm

## EAR-MM feature
- EAR-MM enhances robustness by combining Candidate Searching and Candidate Reusing, improving candidate selection and mitigating noisy GPS data.
- To ensure adaptiveness, EAR-MM uses automatic parameter adjustment with gradient descent to optimize observation and transition probabilities based on changing conditions.
- EAR-MM improves efficiency through a Streaming Inference method that ensures real-time map matching. It accelerates pathfinding with bidirectional Dijkstra and reduces redundant computations using window-limited caching.

## Project Structure
This project mainly includes the following various online trajectory map matching algorithms:

- The main code for the EAR-MM algorithm is in the *org/urbcomp/cupid/db/algorithm/mapmatch/onlinemm* package.
- The main code for the DW-RMM algorithm is in the *org/urbcomp/cupid/db/algorithm/mapmatch/dwrmm* package.
- The main code for the AMM algorithm is in the *org/urbcomp/cupid/db/algorithm/mapmatch/amm* package.
- The main code for the OHMM algorithm is in the *org/urbcomp/cupid/db/algorithm/mapmatch/stream* package.
- The main code for the AOMM algorithm is in the *org/urbcomp/cupid/db/algorithm/mapmatch/aomm* package.

### EAR-MM Structure
EAR-MM consists of three primary modules: _Candidate Preparation_, _Probability Calculation_,
and _Streaming Inference_.

#### Candidate Preparation
The primary objective of this module is to generate candidate locations on the road network for each incoming GPS point in real time. Accurate candidate generation is crucial, as it directly affects subsequent calculations and ultimately the overall accuracy of the map matching process.

#### Probability Calculation
The _Probability Calculation_ module outlines the procedures for calculating the probability of matching observed data to the map, encompassing observation and transition probabilities, as well as automatic parameter adjustment.

#### Streaming Inference
The _Streaming Inference_ module is responsible for processing incoming GPS data in real-time and generating the final matched path.

## Test EAR-MM
The default parameters are set according to the table below and can be adjusted for different data sets.
<table>
  <tr>
    <th>Parameter</th>
    <th>Range</th>
    <th>Default</th>
  </tr>
  <tr>
    <th>sampling Rate r</th>
    <th>origin, 6s, 12s, 30s, 48s, 60s</th>
    <th>6s</th>
  </tr>
  <tr>
    <th>Window Size L</th>
    <th>5, 10, 15, 20, 25</th>
    <th>20</th>
  </tr>
  <tr>
    <th>Weight Parameter</th>
    <th>0.3, 0.4, 0.5, 0.6, 0.7, auto</th>
    <th>auto</th>
  </tr>
</table>

### Prerequisites for testing

The following resources need to be downloaded and installed:

- Java 8 download: https://www.oracle.com/java/technologies/downloads/#java8
- IntelliJ IDEA download: https://www.jetbrains.com/idea/
- git download:https://git-scm.com/download
- maven download: https://archive.apache.org/dist/maven/maven-3/

Download and install jdk-8, IntelliJ IDEA and git. IntelliJ IDEA's maven project comes with maven, you can also use your
own maven environment, just change it in the settings.

### Clone code

1. Open *IntelliJ IDEA*, find the *git* column, and select *Clone...*

2. In the *Repository URL* interface, *Version control* selects *git*

3. URL filling: *###*
4. Due to anonymity restrictions, you can access the code from the following link : *https://anonymous.4open.science/r/StreamingTrajectoryMapMatching-8B8C*

### Set JDK

File -> Project Structure -> Project -> Project SDK -> *add SDK*

Click *JDK* to select the address where you want to download jdk-8.

### Test STEP
Due to permission control, we will only publish part of the Chengdu and Wuxi datasets.
