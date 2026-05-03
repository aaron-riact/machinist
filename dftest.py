# Assuming df.py is in your current directory
from df import DfDecider, DfDecision, DfNetwork

class WorkAction(DfDecider):
    def decide(self):
        # Leaf nodes return None or a specific Action object 
        # to stop the descent
        return None


# 1. Define a Mock Context (The "Mental Model")
class SimpleContext:
    def __init__(self):
        self.battery = 100
        self.has_work = True


# 2. Define a Decider Node
class BatteryCheck(DfDecider):
    def __init__(self):
        super().__init__()
        self.add_child('work', WorkAction())
        self.add_child('charge', WorkAction())

    def decide(self):
        if self.context.battery < 20:
            print("[Decider] Battery low! Charging...")
            return DfDecision("charge")
        
        if self.context.has_work:
            return DfDecision("work")
            
        return DfDecision("idle")

# 3. Running the Network
context = SimpleContext()
print("B", context.battery)
# The DfNetwork wraps a decider and a context
network = DfNetwork(BatteryCheck(), context=context)

# Simulate 5 "ticks" of the robot's brain
for i in range(5):
    context.battery -= 25  # Drain battery each tick
    
    # .step() traces the path from root to leaf
    network.step()
    
    print(f"Tick {i}: Battery={context.battery}% -> Current: {network}") #.value}")

