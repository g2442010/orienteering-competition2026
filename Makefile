CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra

SOURCES  = csv_loader.cpp graph.cpp evaluate.cpp ga.cpp rating.cpp main.cpp
OBJECTS  = $(SOURCES:.cpp=.o)
TARGET   = orienteering

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: clean run
