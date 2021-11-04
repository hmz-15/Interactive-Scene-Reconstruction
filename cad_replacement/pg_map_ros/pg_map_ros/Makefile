#############################################################################
# 'make'        build executable file $(TARGET)
# 'make clean'  removes all .o and executable files
# 'make run'    build executalbe file $(TARGET) and run it ./$(TARGET)
#############################################################################


#############################################################################
#	Define General Setup
#############################################################################

# define thr remove command
RM = rm

# define object files directory
BUILD_DIR := ./obj

# define source file directory
SRC_DIR := ./src

# define binary file directory
BIN_DIR := ./bin

# define test file directory
TEST_DIR := ./tests

# define header file directory
HEADER_DIR := ./include

# define header files
HEADERS := $(wildcard include/*.h)

# define the C/C++ source files
SRC := $(shell find $(SRC_DIR) -name *.cpp)

# define the C/C++ object files
# Here we substitute $(SRC_DIR) prefix with $(BUILD_DIR)
OBJ := $(subst $(SRC_DIR),$(BUILD_DIR),$(SRC:.cpp=.o))

# define dependencies
DEP := ${OBJ:.o=.d}

# define the executable file
EXEC_TEST_NODE := test_nodes
EXEC_TEST_MAP_BUILDER := test_map_builder
EXEC_TEST_SOCKET_CLIENT := test_socket_client
EXEC_ANNOTATE_CG := annotate_cg

#############################################################################
#	Define Compiling Configuration
#############################################################################

# define the C compiler to use
CC = g++

# define any compile-time flags
CFLAGS := -Wall -g -std=c++11

# define any directories containing header files other than /usr/include
# eg: INCLUDES = -I/home/newhall/include  -I../include `pkg-config --cflags opencv`
INCLUDES := -I./$(HEADER_DIR)

# define the searching scope of the library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
#   eg: LDFLAGS = -L/home/newhall/lib  -L../lib `pkg-config --libs opencv`
LDFLAGS := 

# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname 
#   option, something like (this will link in libmylib.so and libm.so:
#   eg: LDLIBS = -lmylib -lm -lboost_program_options
LDLIBS := 


#############################################################################
#	Define Compiling Rules
#############################################################################

all: $(BIN_DIR)/$(EXEC_TEST_NODE) $(BIN_DIR)/$(EXEC_TEST_MAP_BUILDER) \
$(BIN_DIR)/$(EXEC_TEST_SOCKET_CLIENT) $(BIN_DIR)/$(EXEC_ANNOTATE_CG)
	@echo "All executables have been successfully built"
	@echo "$(OK_COLOR)Done, happy coding :)$(NO_COLOR)"


# exec: annotate contact graph
$(BIN_DIR)/$(EXEC_ANNOTATE_CG): $(BUILD_DIR)/annotate_cg.o $(OBJ)
	@echo "Building executable $(OK_COLOR)$@$(NO_COLOR)"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS) $(LDLIBS)
	@echo $(EXEC_ANNOTATE_CG) has been succesfully built


# exec: test_nodes
$(BIN_DIR)/$(EXEC_TEST_NODE): $(BUILD_DIR)/test_nodes.o $(OBJ)
	@echo "Building executable $(OK_COLOR)$@$(NO_COLOR)"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS) $(LDLIBS)
	@echo $(EXEC_TEST_NODE) has been succesfully built


# exec: test_map_builder
$(BIN_DIR)/$(EXEC_TEST_MAP_BUILDER): $(BUILD_DIR)/test_map_builder.o $(OBJ)
	@echo "Building executable $(OK_COLOR)$@$(NO_COLOR)"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS) $(LDLIBS)
	@echo $(EXEC_TEST_MAP_BUILDER) has been succesfully built

# exec: test_socket_client
$(BIN_DIR)/$(EXEC_TEST_SOCKET_CLIENT): $(BUILD_DIR)/test_socket_client.o $(OBJ)
	@echo "Building executable $(OK_COLOR)$@$(NO_COLOR)"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS) $(LDLIBS)
	@echo $(EXEC_TEST_SOCKET_CLIENT) has been succesfully built


$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp $(HEADERS)
	@echo "Compiling $(OBJ_COLOR)$<$(NO_COLOR)"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	@echo ""

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADERS)
	@echo "Compiling $(OBJ_COLOR)$<$(NO_COLOR)"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	@echo ""


# test: $(EXEC)
# 	@$(foreach var, $(TEST_FILES), \
# 		echo "----------------$(var)----------------"; \
# 		./$(EXEC) -c $(var); \
# 	)

clean:
	$(RM) -rf $(BUILD_DIR) $(BIN_DIR)


test: $(BIN_DIR)/$(EXEC_TEST_NODE) $(BIN_DIR)/$(EXEC_TEST_MAP_BUILDER)
	@echo "Run cmd: $(BIN_DIR)/$(EXEC_TEST_NODE)"
	@$(BIN_DIR)/$(EXEC_TEST_NODE)
	@echo "Run cmd: $(BIN_DIR)/$(EXEC_TEST_MAP_BUILDER)"
	@$(BIN_DIR)/$(EXEC_TEST_MAP_BUILDER)

foo: $(BIN_DIR)/$(EXEC_ANNOTATE_CG)
	@echo "Run cmd: $(BIN_DIR)/$(EXEC_ANNOTATE_CG)"
	@$(BIN_DIR)/$(EXEC_ANNOTATE_CG) 223_structure.txt 223_annotated.json


#############################################################################
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#############################################################################
.PHONY: clean run test

COM_COLOR   = \033[0;34m
OBJ_COLOR   = \033[0;36m
OK_COLOR    = \033[0;32m
ERROR_COLOR = \033[0;31m
WARN_COLOR  = \033[0;33m
NO_COLOR    = \033[m

# make dependency
-include $(DEP)