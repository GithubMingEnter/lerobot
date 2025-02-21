import tqdm
seconds = 30 
frequency = 200
for _ in tqdm.tqdm(range(seconds*frequency)):
    leader_pos = robot.leader_arm["main"].read("Present_Position")
    rbot.follower_arm["main"].write("Goal_Position", leader_pos)
    