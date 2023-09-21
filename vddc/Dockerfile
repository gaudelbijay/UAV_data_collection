# Use an ARM64 compatible Python 3 base image
FROM arm64v8/python:3.8-slim

# Set the working directory inside the container
WORKDIR /usr/src/app

# Copy the local python script to the container
COPY meta_data_drone.py .

# Install necessary packages with pip3 (for Python 3)
RUN pip3 install pymavlink numpy

# Command to run when the container starts
CMD ["python3", "./meta_data_drone.py"]
