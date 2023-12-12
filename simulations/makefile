.PHONY: all clean

all: clean test clean

clean:
	rm -f plots/*.png

test:
	# Distance PID
	## Truck
	python simulation.py --ticks=1000 --scenario=1 --vehicles=4 --type=1 --model=1 --period=0.01
	python simulation.py --ticks=1000 --scenario=2 --vehicles=4 --type=1 --model=1 --period=0.01
	python simulation.py --ticks=1000 --scenario=3 --vehicles=4 --type=1 --model=1 --period=0.01
	python simulation.py --ticks=1000 --scenario=4 --vehicles=4 --type=1 --model=1 --period=0.01

	## RC-vehicle
	python simulation.py --ticks=1000 --scenario=1 --vehicles=4 --type=3 --model=1 --period=0.01
	python simulation.py --ticks=1000 --scenario=2 --vehicles=4 --type=3 --model=1 --period=0.01
	python simulation.py --ticks=1000 --scenario=3 --vehicles=4 --type=3 --model=1 --period=0.01

	# Bidirectional PID
	## Truck
	python simulation.py --ticks=1000 --scenario=1 --vehicles=4 --type=1 --model=2 --period=0.1
	python simulation.py --ticks=1000 --scenario=2 --vehicles=4 --type=1 --model=2 --period=0.1
	python simulation.py --ticks=1000 --scenario=3 --vehicles=4 --type=1 --model=2 --period=0.1

	# Bidirectional State Space
	## Truck
	python simulation.py --ticks=1000 --scenario=1 --vehicles=4 --type=1 --model=3 --period=0.1
	python simulation.py --ticks=1000 --scenario=2 --vehicles=4 --type=1 --model=3 --period=0.1
	python simulation.py --ticks=1000 --scenario=3 --vehicles=4 --type=1 --model=3 --period=0.1

	## RC-vehicle
	python simulation.py --ticks=1000 --scenario=1 --vehicles=4 --type=3 --model=3 --period=0.1
	python simulation.py --ticks=1000 --scenario=2 --vehicles=4 --type=3 --model=3 --period=0.1
	python simulation.py --ticks=1000 --scenario=3 --vehicles=4 --type=3 --model=3 --period=0.1

# If any command fails, stop the entire make process
.SILENT: ; # No need for echoing commands