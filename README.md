README 

This was a particularly challenging but rewarding project. The collisions functionality involved a lot of work with linear algebra and physics formula using a whiteboard and test/re-testing. 

Normal Vector Calculation: Different obstacles have different shapes so that we have to design collision mechanism for each of them. Particularly for rectangular shapes (blocks and flippers), we have to calculate the outer boundary (four lines in our case) to detect the collision. If the speed of the ball is large, it will pass through the collision boundary and trigger the bug, as mentioned by Prof. Ridge on Piazza. 
Black Hole Teleportation: If the ball enters the black ball, the ball will be teleported to the top, thus increasing the probability for the ball to hit obstacles and earn points. 
