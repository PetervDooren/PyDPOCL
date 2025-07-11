import random
import uuid

# Set a fixed seed for repeatability across executions
_rng = random.Random(42)

def duuid4(): # deterministic uuid4
    # Generate a deterministic 128-bit integer
    return uuid.UUID(int=_rng.getrandbits(128))

if __name__ == '__main__':
    # Example usage:
    for _ in range(3):
        print(duuid4())