# Makefile for Docker management
IMAGE_NAME=collab-3d-printing-sim
CONTAINER_NAME=collab_3d_printing_container
WS=/root/collab_3d_printing_ws

# Build Docker image
build:
	docker build -t $(IMAGE_NAME) .

# Run Docker container with X11 forwarding for Gazebo GUI
run:
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--env="DISPLAY=${DISPLAY}" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--net=host \
		-v $(PWD)/ws:/root/collab_3d_printing_ws \
		$(IMAGE_NAME) bash

# Clean workspace and Docker artifacts
clean:
	sudo rm -rf $(WS)/build $(WS)/install $(WS)/log
	docker rm -f $(CONTAINER_NAME) || true

# Rebuild without cache
rebuild: clean
	docker build --no-cache -t $(IMAGE_NAME) .

# Stop container
stop:
	docker stop $(CONTAINER_NAME) || true
