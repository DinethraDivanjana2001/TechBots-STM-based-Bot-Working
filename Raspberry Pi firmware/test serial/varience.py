import statistics

# Read the data from the txt file
with open('output3.txt', 'r') as file:
    data = [int(line.strip()) for line in file.readlines()]

# Calculate the variance
variance = statistics.variance(data)

print("Variance:", variance)
