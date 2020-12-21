# CS454-Final-Project
K-DAsFault: Kaistian-Designed Automatically Testing Self-Driving Cars with Search-Based Procedural Content Generation

# Team information
## Team 21
- 20160633 Cho Jaemin
- 20170057 Geon Kim
- 20170150 Jaeuk Kim
- 20170168 Jiin Kim
- 20170719 Han Seunghee

# Repository description
- KDAsFault : Our K-DAsFault model.
- Team21_Final Report : Final report of our project.
- Team21_Final Presentation : Slides of our final presentation.

# How to run the code
1. Go to AD_Simul folder
```bash
cd KDAsFault
cd AD_Simul
```
2. Open LKA_Test.m
3. Execute LKA_Test.m
```matlab
LKA_Test
```
4. Go to default folder (KDAsFault)
```bash
cd KDAsFault
```
5. Execute GA.py
```bash
python GA.py
```

# Excute options (GA.py)

## Parameters
-p : population size (default = 25)

-f : generation number (default = 40)

-k : selection size (default = 5)

-e : elitism (default = 0.1)

-cr : crossover method (default = 'join')

-x : map x size (default = 200, small = 200 / large = 400)

-y : map y size (default = 200, small = 200 / large = 400)

-w : map road width (default = 4)

-n : path geneartion number (default = 3)

## Road number option
To test for a single road, comment 337 line of Road.py and uncomment 338 line of Road.py.
```python
# road_number = random.choice([2,3])
road_number = 1
```
To test for a multi road, comment 338 line of Road.py and uncomment 337 line of Road.py.
```python
road_number = random.choice([2,3])
# road_number = 1
```
(default = multi)

# Project Proposal Video
Available at: https://youtu.be/xZsIdBGCrSQ

# Project Final Presentation Video
Available at: https://youtu.be/jB8fgKgRP9g
