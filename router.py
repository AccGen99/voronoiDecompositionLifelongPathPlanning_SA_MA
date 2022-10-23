class route:
    def __init__(self, ID, start_node):
        self.ID = ID
        self.visited = [start_node]
        self.time_costs = []
        self.partition = None

    def add_partition(self, part):
        self.partition = part

    def add_node_time(self, next_node, duration):
        self.visited.append(next_node)
        self.time_costs.append(duration)
