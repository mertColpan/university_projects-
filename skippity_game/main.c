

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#define MAX 5
#define MAX_ITEM 50

typedef struct{
	int x;
	int y;
}Cordinate;

typedef struct {
	Cordinate items[MAX_ITEM];
	int top;
}STACK;


void initialize(STACK *s){
	s->top = 0;
}

int isEmpty(STACK *s){	
	if(s->top == 0){	
		return 1;	
	}
	return 0;
}

int isFull(STACK *s){
	if(s->top == MAX_ITEM){
		
		return 1;	
	}
	return 0;
}

int push(STACK *s, Cordinate c){
	if(isFull(s)){	
		return 0;	
	}
	else{	
		s->items[s->top] = c;
		(s->top)++;
		return 1;
	}
}

int pop (STACK *s, Cordinate *c){
	if(isEmpty(s)){	
		return 0;	
	}
	else{	
		(s->top)--;
		*c = s->items[s->top];
		return 1;
	}
}


/*
 @brief Creates a game board with random values and initializes the center cells to zero.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 */
void createBoard(int **board, int dim){
    srand(time(NULL)); // Seed random number generator with current time
    int i, j;
    int random;
    for(i=0;i<dim;i++){
        for(j=0;j<dim;j++){
            random = (rand()%5) + 1; // Generate a random number from 1 to 5
            board[i][j] = random; // Assign the random number to the current cell
        }
    }
    // Initialize center cells to zero
    board[dim/2][dim/2] = 0;
    board[dim/2-1][dim/2] = 0;
    board[dim/2-1][dim/2-1] = 0;
    board[dim/2][dim/2-1] = 0;
}


/*
 @brief Displays the game board with appropriate labels and symbols.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 */
void displayBoard(int **board, int dim){
    int i, j;
    printf("     ");
    for(i = 0; i < dim; i++){
        printf("%d   ", i); // Print column labels
    }
    printf("\n");
    for(i = 0; i < dim; i++){
        printf(" %d |", i); // Print row labels
        for(j = 0; j < dim; j++){
            if(board[i][j] != 0){
                printf(" %c |", (64 + board[i][j])); // Print cell value with appropriate symbol
            }	
            else{
                printf("   |"); // Print empty cell
            }
        }
        printf("\n\n");
    }
}


/*
 @brief Checks if the game has reached its end by examining neighboring cells.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 
 @return int Returns 1 if the game has ended, otherwise returns 0.
 */
int isGameEnd(int **board, int dim){
    int i, j;
    
    for(i = 0; i < dim; i++){
        for(j = 0; j < dim; j++){
            if(board[i][j] != 0){
                // Check left neighbors
                if(((j - 2) >= 0) && (board[i][j - 1] != 0)){
                    if(board[i][j - 2] == 0){
                        return 0; // Game is not ended
                    }
                }
                // Check right neighbors
                if(((j + 2) < dim) && (board[i][j + 1] != 0)){
                    if(board[i][j + 2] == 0){
                        return 0; // Game is not ended
                    }
                }
                // Check upper neighbors
                if(((i - 2) >= 0) && (board[i - 1][j] != 0)){
                    if(board[i - 2][j] == 0){
                        return 0; // Game is not ended
                    }
                }
                // Check lower neighbors
                if(((i + 2) < dim) && (board[i + 1][j] != 0)){
                    if(board[i + 2][j] == 0){
                        return 0; // Game is not ended
                    }
                }
            }	
        }
    }
    return 1; // Game has ended
}


/*
 @brief Calculates the scores of players and the number of extra skippers.
 
 This function calculates the scores of two players based on their skipper counts in each round
 and also calculates the number of extra skippers each player has compared to the minimum skipper count.
 
 @param players The two-dimensional array representing the skipper counts of each player in each round.
 @param player2Score Pointer to the variable storing the score of player 2.
 @param player1Score Pointer to the variable storing the score of player 1.
 @param player1ExtraSkippers Pointer to the variable storing the number of extra skippers for player 1.
 @param player2ExtraSkippers Pointer to the variable storing the number of extra skippers for player 2.
 */
void calculateScore(int players[][MAX], int *player2Score, int *player1Score, int *player1ExtraSkippers, int *player2ExtraSkippers){
    int player1Min = players[0][0], player2Min = players[1][0]; // Initialize minimum skipper counts
    int i; 
    // Find minimum skipper count for each player
    for(i = 1; i < MAX; i++){
        if(player1Min > players[0][i]){
            player1Min = players[0][i];
        }
        if(player2Min > players[1][i]){
            player2Min = players[1][i];
        }
    }
    // Calculate extra skippers for each player
    for(i = 1; i < MAX; i++){
        if(players[0][i] > player1Min){
            *player1ExtraSkippers += players[0][i] - player1Min;
        }
        if(players[1][i] > player2Min){
            *player2ExtraSkippers += players[1][i] - player2Min;
        }
    }
    // Assign minimum skipper counts to player scores
    *player2Score = player2Min;
    *player1Score = player1Min;
}

/*
 @brief Displays the scores and skipper counts of players.
 
 This function displays the skipper counts of each player in each round,
 their scores, and the number of extra skippers each player has compared to the minimum skipper count.
 
 @param players The two-dimensional array representing the skipper counts of each player in each round.
 */
void displayScore(int players[][MAX]){
    // Initialize variables to store scores and extra skippers
    int player1Score, player2Score, player1ExtraSkippers = 0, player2ExtraSkippers = 0;
    // Calculate scores and extra skippers using the calculateScore function
    calculateScore(players, &player2Score, &player1Score, &player1ExtraSkippers, &player2ExtraSkippers);
    int i;
    printf("Player 1 skippers: ");
    // Print skipper counts for player 1
    for(i = 0; i < MAX; i++){
        printf(" %d", players[0][i]);
    }
    printf("  Player 2 skippers: ");
    // Print skipper counts for player 2
    for(i = 0; i < MAX; i++){
        printf(" %d", players[1][i]);
    }
    printf("\n                   ");
    // Print labels for skipper counts
    for(i = 0; i < MAX; i++){
        printf(" %c", 65 + i);
    }
    printf("                     ");
    // Print labels for skipper counts
    for(i = 0; i < MAX; i++){
        printf(" %c", 65 + i);
    }
    printf("\n\nPlayer Score: %d           Player 2 Score: %d", player1Score, player2Score);
    printf("\n\nPlayer 1 extra skippers: %d     Player 2 extra skippers: %d", player1ExtraSkippers, player2ExtraSkippers);
}


/*
 @brief Reads a game board from a file and stores it in a two-dimensional array.
 
 This function reads a game board from a file named "gameBoard3.txt" and stores it in a dynamically allocated
 two-dimensional array. It also updates the dimension of the game board.
 
 @param board Pointer to the two-dimensional array representing the game board.
 @param dim Pointer to the variable storing the dimension of the game board.
 
 @return int** Pointer to the dynamically allocated two-dimensional array storing the game board.
 */
int **readFromFileGameBoard(int **board, int *dim){
    FILE *file;
    int row = 0, column = 0;
    char c;
    
    // Allocate memory for the two-dimensional array
    board = (int**)malloc(20 * sizeof(int*));
    int i;
    for(i = 0; i < 20; i++){
        board[i] = (int*)malloc(20 * sizeof(int));
    }
    
    // Open the file for reading
    file = fopen("gameBoard3.txt", "r");
    if (file == NULL) {
        printf("File didn't open.\n");
        exit(1);
    }
    
    // Read the game board from the file
    while ((c = fgetc(file)) != EOF) {
        if (c == '\n') {
            row++;
            column = 0;
        }
        else {
            board[row][column] = c - '0';
            column++;	
        }
    }
    fclose(file);

    // Update the dimension of the game board
    *dim = row;
    
    return board;
}


/*
 @brief Reads game information from a file and updates game variables.
 
 This function reads game information from a file named "info3.txt" and updates the priority player,
 skipper counts of players, and the game type based on the information read from the file.
 
 @param priorityPlayer Pointer to the variable storing the priority player.
 @param players Two-dimensional array storing the skipper counts of players.
 @param gameType Pointer to the variable storing the game type.
 */
void readFromFileGameInfo(int *priorityPlayer, int players[][MAX], int *gameType){
    FILE *file;
    char c;
    int matrix[12];
    int index = 0;
    
    // Open the file for reading
    file = fopen("info3.txt", "r");
    if (file == NULL) {
        printf("File didn't open.\n");
        exit(1);
    }
    
    // Read game information from the file
    while ((c = fgetc(file)) != EOF) {
        matrix[index++] = c - '0'; 
    }
    fclose(file);
    
    // Update priority player, skipper counts of players, and game type
    *priorityPlayer = matrix[0];
    int i;
    for(i = 1; i < 6; i++){
        players[0][i - 1] = matrix[i];
    }
    for(i = 6; i < 11; i++){
        players[1][i - 6] = matrix[i];
    }
    *gameType = matrix[11];
}

/*
 @brief Saves the game board to a file.
 
 This function saves the game board represented by a two-dimensional array to a file named "gameBoard.txt".
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 */
void saveGameBoardToFile(int **board, int dim){
    FILE *file = fopen("gameBoard.txt", "w");
    
    // Check if file is opened successfully
    if(file == NULL){
        printf("File didn't open");
        exit(1);
    }
    
    // Write game board to the file
    int i, j;
    for(i = 0; i < dim; i++){
        for(j = 0; j < dim; j++){
            fprintf(file, "%d", board[i][j]);
        }
        fprintf(file, "\n");
    }
    fclose(file);
}


/*
 @brief Saves the game information to a file.
 
 This function saves the game information including priority player, skipper counts of players,
 and game type to a file named "info.txt".
 
 @param priorityPlayer The priority player.
 @param players Two-dimensional array storing the skipper counts of players.
 @param gameType The type of the game.
 */
void saveGameInfoToFile(int priorityPlayer, int players[][MAX], int gameType) {
    FILE *file = fopen("info.txt", "w");
    
    // Check if file is opened successfully
    if (file == NULL) {
        printf("File didn't open");
        exit(1);
    }
    
    // Write game information to the file
    fprintf(file, "%d", priorityPlayer);
    int i;
    for (i = 0; i < 5; i++) {
        fprintf(file, "%d", players[0][i]);
    }
    for (i = 0; i < 5; i++) {
        fprintf(file, "%d", players[1][i]);
    }
    fprintf(file, "%d", gameType);
    fprintf(file, "\n");
    fclose(file);
}


/*
 @brief Checks if a move is legal on the game board.
 
 This function checks if a move from source coordinate to destination coordinate is legal on the game board.
 A move is considered legal if the source and destination coordinates are adjacent, and the destination
 coordinate is empty. Additionally, if the move is a skip move, the coordinate of the skipped position is also returned.
 
 @param board The two-dimensional array representing the game board.
 @param src The source coordinate of the move.
 @param dest The destination coordinate of the move.
 @param dim The dimension of the game board.
 @param c3 Pointer to the coordinate of the skipped position (if applicable).
 
 @return int Returns 1 if the move is legal, otherwise returns 0.
 */
int isMoveLegal(int **board, Cordinate src, Cordinate dest, int dim, Cordinate *c3){
    if(src.x != dest.x){ // Horizontal move
        if((src.y >= 0) && (src.y < dim)){
            if((src.x >= 0) && (src.x < dim)){
                if((dest.x >= 0) && (dest.x < dim)){
                    if((board[src.x][src.y] != 0) && (board[dest.x][dest.y] == 0) ){
                        if(src.x == (dest.x + 2)){ // Skip move
                            c3->x = dest.x + 1;
                            c3->y = dest.y;
                            if(board[src.x - 1][src.y] != 0){
                                return 1;
                            }
                        }
                        else if(dest.x == (src.x + 2)){ // Skip move
                            c3->x = src.x + 1;
                            c3->y = dest.y;
                            if(board[dest.x - 1][src.y] != 0){
                                return 1;
                            }
                        }
                    }
                }
            }
        }
    }
    else{ // Vertical move
        if((src.x >= 0) && (src.x < dim)){
            if((src.y >= 0) && (src.y < dim)){
                if((dest.y >= 0) && (dest.y < dim)){
                    if((board[src.x][src.y] != 0) && (board[dest.x][dest.y] == 0) ){
                        if(src.y == (dest.y + 2)){ // Skip move
                            c3->x = src.x;
                            c3->y = dest.y + 1;
                            if(board[src.x][src.y - 1] != 0){
                                return 1;
                            }
                        }
                        else if(dest.y == (src.y + 2)){ // Skip move
                            c3->x = src.x;
                            c3->y = src.y + 1;
                            if(board[src.x][dest.y - 1] != 0){
                                return 1;
                            }
                        }
                    }
                }
            }
        }
    }
    return 0; // Move is not legal
}


/*
 @brief Makes a move on the game board.
 
 This function makes a move from source coordinate to destination coordinate on the game board.
 It also updates the value of the last skipper taken during the move.
 
 @param board The two-dimensional array representing the game board.
 @param src The source coordinate of the move.
 @param dest The destination coordinate of the move.
 @param taken Pointer to the coordinate of the skipped position.
 @param lastSkipper Pointer to the variable storing the value of the last skipper taken.
 */
void makeMove(int **board, Cordinate src, Cordinate dest, Cordinate *taken, int *lastSkipper){
    int value;
    // Get the value of the skipper at the source coordinate
    value = board[src.x][src.y];
    // Place the skipper at the destination coordinate
    board[dest.x][dest.y] = value;
    // Update the value of the last skipper taken
    *lastSkipper = board[taken->x][taken->y];
    // Clear the skipper from the taken coordinate
    board[taken->x][taken->y] = 0;
    // Clear the source coordinate
    board[src.x][src.y] = 0;
}

/*
 @brief Resets the game board to its initial state.
 
 This function resets the game board by setting all elements to zero, effectively clearing the board.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 
 @return int** Pointer to the reset game board.
 */
int **resetBoard(int **board, int dim){
    int i, j;
    // Set all elements of the game board to zero
    for(i = 0; i < dim; i++){
        for(j = 0; j < dim; j++){
            board[i][j] = 0;
        }
    }
    return board;
}


/*
 @brief Checks if there are any possible moves for a skipper on the game board.
 
 This function checks if there are any possible moves for the skipper at the specified coordinate on the game board.
 
 @param board The two-dimensional array representing the game board.
 @param c The coordinate of the skipper.
 @param dim The dimension of the game board.
 
 @return int Returns 1 if there are possible moves for the skipper, otherwise returns 0.
 */
int isAnyMoves(int **board, Cordinate c, int dim){
    if(board[c.x][c.y] != 0){ // Check if there is a skipper at the specified coordinate
        if(((c.y - 2) >= 0) && (board[c.x][c.y - 1] != 0)){ // Check if there is a skipper to the left
            if(board[c.x][c.y - 2] == 0){ // Check if the position two spaces to the left is empty
                return 1;
            }
        }
        if(((c.y + 2) < dim) && (board[c.x][c.y + 1] != 0)){ // Check if there is a skipper to the right
            if(board[c.x][c.y + 2] == 0){ // Check if the position two spaces to the right is empty
                return 1;
            }
        }
        if(((c.x - 2) >= 0) && (board[c.x - 1][c.y] != 0)){ // Check if there is a skipper above
            if(board[c.x - 2][c.y] == 0){ // Check if the position two spaces above is empty
                return 1;
            }
        }
        if(((c.x + 2) < dim) && (board[c.x + 1][c.y] != 0)){ // Check if there is a skipper below
            if(board[c.x + 2][c.y] == 0){ // Check if the position two spaces below is empty
                return 1;
            }
        }   
    }
    return 0; // No possible moves for the skipper
}


/*
 @brief Copies the contents of one game board to another.
 
 This function copies the contents of one game board to another.
 
 @param board The two-dimensional array representing the original game board.
 @param copy The two-dimensional array representing the copy of the game board.
 @param dim The dimension of the game board.
 */
void boardCopy(int **board, int **copy, int dim){
    int i, j;
    // Copy the contents of the original game board to the copy game board
    for(i = 0; i < dim; i++){
        for(j = 0; j < dim; j++){
            copy[i][j] = board[i][j];
        }
    }
}

/*
 @brief Deletes a skipper from a player's skipper count.
 
 This function decrements the skipper count of a player corresponding to the specified skipper value.
 
 @param priorityPlayer The priority player whose skipper count will be decremented.
 @param player Two-dimensional array storing the skipper counts of players.
 @param skipper The value of the skipper to be deleted.
 */
void deleteSkipper(int priorityPlayer, int player[][MAX], int skipper){
    int i;
    // Decrement the skipper count of the priority player corresponding to the specified skipper value
    for(i = 0; i < MAX; i++){
        if((i + 1) == skipper){
            player[priorityPlayer][i]--;
        }
    }
}

/*
 @brief Adds a skipper to a player's skipper count.
 
 This function increments the skipper count of a player corresponding to the specified skipper value.
 
 @param priorityPlayer The priority player whose skipper count will be incremented.
 @param player Two-dimensional array storing the skipper counts of players.
 @param skipper The value of the skipper to be added.
 */
void addSkipper(int priorityPlayer, int player[][MAX], int skipper){
    int i;
    // Increment the skipper count of the priority player corresponding to the specified skipper value
    for(i = 0; i < MAX; i++){
        if((i + 1) == skipper){
            player[priorityPlayer][i]++;
        }
    }
}


/*
 @brief Determines the winner of the game.
 
 This function determines the winner of the game based on the skipper counts of the players.
 It compares the minimum skipper count of each player and their extra skippers to decide the winner.
 
 @param players Two-dimensional array storing the skipper counts of players.
 */
void getWinner(int players[][MAX]){
    // Initialize variables to store minimum skipper count and extra skippers for each player
    int player1Min = players[0][0], player2Min = players[1][0];
    int player1ExtraSkippers = 0, player2ExtraSkippers = 0;
    int i; 
    // Find the minimum skipper count for each player
    for(i = 1; i < MAX; i++){
        if(player1Min > players[0][i]){
            player1Min = players[0][i];
        }
        if(player2Min > players[1][i]){
            player2Min = players[1][i];
        }
    }
    // Calculate extra skippers for each player
    for(i = 1; i < MAX; i++){
        if(players[0][i] > player1Min){
            player1ExtraSkippers += players[0][i] - player1Min;
        }
        if(players[1][i] > player2Min){
            player2ExtraSkippers += players[1][i] - player2Min;
        }
    }
    // Determine the winner based on skipper counts and extra skippers
    if(player1Min < player2Min){
        printf("Player 2 won");
    }
    else if(player1Min == player2Min){
        if(player1ExtraSkippers < player2ExtraSkippers){
            printf("Player 2 won");
        }
        else if(player1ExtraSkippers == player2ExtraSkippers){
            printf("DRAW");
        }
        else{
            printf("Player 1 won");
        }
    }
    else{
        printf("Player 1 won");
    }
}

/*
 @brief Implements the gameplay for a player versus player mode.
 
 This function allows two players to play against each other in the game.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 @param priorityPlayer The priority player who starts the game.
 @param players Two-dimensional array storing the skipper counts of players.
 */
void playerVsPlayer(int **board, int dim, int priorityPlayer, int players[][MAX]){
    // Initialize variables and allocate memory for temporary boards and coordinates
    Cordinate currentSkipper, destination, *taken;
    taken = (Cordinate*)malloc(sizeof(Cordinate));
    int **previousBoard = (int**)malloc(dim * sizeof(int*));
    if(previousBoard == NULL){
        printf("Memory Allocation Error");
        exit(1);
    }
    int i;
    for(i = 0; i < dim; i++){
        previousBoard[i] = (int*)malloc(dim * sizeof(int));
    }
    int **tmpBoard = (int**)malloc(dim * sizeof(int*));
    if(tmpBoard == NULL){
        printf("Memory Allocation Error");
        exit(1);
    }
    for(i = 0; i < dim; i++){
        tmpBoard[i] = (int*)malloc(dim * sizeof(int));
    }
    int lastSkipper;
    int turn = priorityPlayer;
    char decision;
    int flag = 0;
    int counter = 0;
    
    // Main game loop
    while(!isGameEnd(board, dim) && (!flag)){
        // Copy the current board to previousBoard for undo functionality
        boardCopy(board, previousBoard, dim);
        system("cls");
        displayBoard(board, dim);
        printf("\n");
        displayScore(players);
        printf("\n\nPlayer%d: Choose skipper: x: ", turn + 1);
        scanf("%d", &currentSkipper.x);
        printf("y: ");
        scanf("%d", &currentSkipper.y);
        
        // Check if there are any possible moves for the selected skipper
        while(isAnyMoves(board, currentSkipper, dim)){
            if(counter != 0){
                displayBoard(board, dim);
                displayScore(players);
                printf("\n");
            }
            printf("Choose destination coordinate x: ");
            scanf("%d", &destination.x);
            printf("y: ");
            scanf("%d", &destination.y);
            
            // Check if the move is legal
            if(isMoveLegal(board, currentSkipper, destination, dim, taken)){
                // Update skipper counts and make the move
                players[turn][board[taken->x][taken->y] - 1]++;
                makeMove(board, currentSkipper, destination, taken, &lastSkipper);
                currentSkipper.x = destination.x;
                currentSkipper.y = destination.y;
                counter++;
                
                // Prompt the player for save, undo, or continue options
                printf("\n\nSave and Quit: [Q/q] Undo: [U/u] Continue: [C/c]");
                scanf(" %c", &decision);
                
                // Process player's decision
                if((decision == 'Q') || (decision == 'q')){
                    saveGameBoardToFile(board, dim);
                    saveGameInfoToFile(1, players, 0);
                    flag = 1;
                }
                else if((decision == 'U') || (decision == 'u') ){
                    system("cls");
                    displayBoard(board, dim);
                    displayScore(players);
                    boardCopy(board, tmpBoard, dim);
                    boardCopy(previousBoard, board, dim);
                    deleteSkipper(turn, players, lastSkipper);
                    displayBoard(board, dim);
                    displayScore(players);
                    printf("\n\nSave and Quit: [Q/q] Redo: [R/r] Continue: [C/c]");
                    scanf(" %c", &decision);
                    
                    if((decision == 'Q') || (decision == 'q')){
                        saveGameBoardToFile(board, dim);
                        saveGameInfoToFile(1, players, 0);
                        flag = 1;
                    }
                    else if((decision == 'R') || (decision == 'r')){
                        boardCopy(tmpBoard, board, dim);
                        addSkipper(turn, players, lastSkipper);
                        if(turn == 0){
                            turn++;
                        }
                        else{
                            turn = 0;
                        }
                    }
                }
            }
            else{
                printf("\nInvalid move.\n");
            }
        }
        counter = 0;
        
        // Switch turns between players
        if(turn == 0){
            turn++;
        }
        else{
            turn = 0;
        }
    }
    
    // Free dynamically allocated memory
    for(i = 0; i < dim; i++) {
        free(tmpBoard[i]);
        free(previousBoard[i]);
    }
    free(tmpBoard);
    free(previousBoard);
    free(taken);
    
    // If the game is not ended by a flag, display the winner
    if(!flag){
        system("cls");
        displayScore(players);
        printf("\n");
        getWinner(players);
    }
}


/*
 @brief Reverts the board to its previous state after a move.
 
 This function reverts the board to its previous state by moving the skipper back to its original position,
 placing the taken skipper back, and clearing the destination square.
 
 @param board The two-dimensional array representing the game board.
 @param src The source coordinate of the skipper before the move.
 @param taken The coordinate of the skipper that was taken during the move.
 @param dest The destination coordinate of the skipper before the move.
 @param lastSkipper The value of the skipper that was taken during the move.
 
 @return int** Pointer to the modified game board.
 */
int** backBoard(int **board, Cordinate src, Cordinate taken, Cordinate dest, int lastSkipper) {
    // Move the skipper back to its original position
    board[src.x][src.y] = board[dest.x][dest.y];
    // Place the taken skipper back
    board[taken.x][taken.y] = lastSkipper;
    // Clear the destination square
    board[dest.x][dest.y] = 0;
    return board;
}


/*
 @brief Finds the optimal move for the computer player using backtracking.
 
 This function recursively explores possible moves for the computer player using backtracking.
 It utilizes a stack to keep track of the moves made and finds the move sequence with the highest
 number of skips.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 @param src The source coordinate of the skipper for the current move.
 @param dest The destination coordinate of the skipper for the current move.
 @param taken The coordinate of the skipper that was taken during the current move.
 @param lastSkipper The value of the skipper that was taken during the current move.
 @param moves Array to store the optimal move sequence.
 @param s Stack to keep track of the moves made.
 @param counter Pointer to the counter for the current move sequence.
 @param maxCounter Pointer to the maximum counter for the optimal move sequence.
 */
void findComputerMove(int **board, int dim, Cordinate src, Cordinate dest, Cordinate taken, int lastSkipper, Cordinate moves[MAX_ITEM], STACK *s, int *counter, int *maxCounter) {
    // Define directions for potential moves
    Cordinate directions[] = {
        {src.x, src.y+2},
        {src.x+2, src.y},
        {src.x, src.y-2},
        {src.x-2, src.y}
    };

    // Iterate through each direction
    int i, j;
    for (i = 0; i < 4; i++) {
        // Check if the move is legal
        if (isMoveLegal(board, src, directions[i], dim, &taken)) {
            // Push the move to the stack
            if (!push(s, src) || !push(s, directions[i])) {
                printf("Stack push error\n");
                return;
            }

            (*counter)++;
            // Make the move
            makeMove(board, src, directions[i], &taken, &lastSkipper);
            
            // Check if the current move sequence is longer than the maximum counter
            if (*counter > *maxCounter) {
                // Update the optimal move sequence
                for (j = 0; j < (*counter) * 2; j++) {
                    moves[j] = s->items[j];
                }
                *maxCounter = *counter;
            }

            // Recursively explore the next move
            findComputerMove(board, dim, directions[i], dest, taken, lastSkipper, moves, s, counter, maxCounter);

            // Undo the move
            board = backBoard(board, src, taken, directions[i], lastSkipper);
            (*counter)--;
            
            // Pop the move from the stack
            if (!pop(s, &dest) || !pop(s, &src)) {
                printf("Stack pop error\n");
                return;
            } 
        }
    }
}




/*
 @brief Finds the optimal computer move and executes it.
 
 This function finds the optimal move for the computer player using backtracking.
 It then executes the found move on the game board and updates the player's skipper count.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 @param players The array representing the players' skipper counts.
 
 @return int** Pointer to the modified game board after the computer's move.
 */
int** computerMove(int **board, int dim, int players[][MAX]) {
    int i, j;
    Cordinate current;
    Cordinate taken;
    int lastSkipper;
    // Create and initialize a stack for backtracking
    STACK *s = (STACK*)calloc(1, sizeof(STACK));
    if (s == NULL) {
        printf("Memory Allocation Error");
        exit(1);
    }
    initialize(s);
    // Array to store the optimal move sequence
    Cordinate moves[MAX_ITEM];
    Cordinate dest;
    // Variables to keep track of the maximum counter and current counter
    int maxCounter = -1;
    int counter = 0;
    // Iterate through each cell on the board
    for (i = 0; i < dim; i++) {
        for (j = 0; j < dim; j++) {
            current.x = i;
            current.y = j;
            // Check if the current cell contains a skipper
            if (isAnyMoves(board, current, dim)) {
                // Find the optimal move sequence for the current skipper
                findComputerMove(board, dim, current, dest, taken, lastSkipper, moves, s, &counter, &maxCounter);
            }
        }
    }

    // Print the optimal move sequence
    printf("\n");
    for (i = 0; i < maxCounter * 2; i += 2) {
        printf("%d. move src: %d, %d\n", (i / 2 + 1), moves[i].x, moves[i].y);
        printf("%d. move dest: %d, %d\n", (i / 2 + 1), moves[i + 1].x, moves[i + 1].y);
    }
    // Execute the optimal move sequence
    for (i = 0; i < maxCounter * 2; i += 2) {
        // Determine the coordinate of the taken skipper
        if (moves[i].x != moves[i + 1].x) {
            taken.x = (moves[i].x + moves[i + 1].x) / 2;
            taken.y = moves[i].y;
        } else {
            taken.y = (moves[i].y + moves[i + 1].y) / 2;
            taken.x = moves[i].x;
        }
        // Make the move and update the skipper count
        makeMove(board, moves[i], moves[i + 1], &taken, &lastSkipper);
        addSkipper(1, players, lastSkipper);
    }

    return board;
}


/*
 @brief Player versus Computer game mode.
 
 This function simulates a game where a human player competes against the computer. 
 The player and the computer take turns making moves on the game board until one of them wins or the game ends in a draw.
 
 @param board The two-dimensional array representing the game board.
 @param dim The dimension of the game board.
 @param priorityPlayer The index of the player who has the priority to make the first move.
 @param players The array representing the players' skipper counts.
 */
void playerVsComputer(int **board, int dim, int priorityPlayer, int players[][MAX]){
    Cordinate currentSkipper, destination, taken;
    int **previousBoard = (int**)malloc(dim*sizeof(int*));
    if(previousBoard == NULL){
        printf("Memory Allocation Error");
        exit(1);
    }
    int i;
    for(i=0;i<dim;i++){
        previousBoard[i] = (int*)malloc(dim*sizeof(int));
    }
    int **tmpBoard = (int**)malloc(dim*sizeof(int*));
    if(tmpBoard == NULL){
        printf("Memory Allocation Error");
        exit(1);
    }
    for(i=0;i<dim;i++){
        tmpBoard[i] = (int*)malloc(dim*sizeof(int));
    }
    int lastSkipper;
    int turn = priorityPlayer;
    char decision;
    int flag = 0;
    int counter = 0;
    // Main game loop
    while(!isGameEnd(board, dim) && (!flag)){
        // Copy the current board for undo functionality
        boardCopy(board, previousBoard, dim);
        
        // Display the current game board and scores
        displayBoard(board, dim);
        printf("\n");
        displayScore(players);
        
        // Human player's turn
        if(turn == 0){
            printf("\n\nPlayer%d: Choose skipper: x: ",turn+1);
            scanf("%d",&currentSkipper.x );
            printf("y: ");
            scanf("%d", &currentSkipper.y);
            // Validate the selected skipper and destination
            while(isAnyMoves(board, currentSkipper, dim)){
                if(counter != 0){
                    displayBoard(board, dim);
                    displayScore(players);
                    printf("\n");
                }
                printf("Choose destination coordinate x: ");
                scanf("%d",&destination.x );
                printf("y: ");
                scanf("%d", &destination.y);
                // If the move is legal, make the move
                if(isMoveLegal(board, currentSkipper, destination, dim, &taken)){
                    players[turn][board[taken.x][taken.y] - 1]++;
                    makeMove(board, currentSkipper, destination, &taken, &lastSkipper);
                    currentSkipper.x = destination.x;
                    currentSkipper.y = destination.y;
                    counter++;
                    printf("\n\nSave and Quit: [Q/q] Undo: [U/u] Continue: [C/c]");
                    scanf(" %c", &decision);
                    // Process the player's decision
                    if((decision == 'Q') || (decision == 'q')){
                        saveGameBoardToFile(board, dim);
                        saveGameInfoToFile(1, players, 0);
                        flag = 1;
                    }
                    else if((decision == 'U') || (decision == 'u') ){
                        // Undo the last move
                        system("cls");
                        displayBoard(board, dim);
                        displayScore(players);
                        boardCopy(board, tmpBoard, dim);
                        boardCopy(previousBoard, board, dim);
                        deleteSkipper(turn, players, lastSkipper);
                        displayBoard(board, dim);
                        displayScore(players);
                        printf("\n\nSave and Quit: [Q/q] Redo: [R/r] Continue: [C/c]");
                        scanf(" %c", &decision);
                        
                        if((decision == 'Q') || (decision == 'q')){
                        saveGameBoardToFile(board, dim);
                        saveGameInfoToFile(1, players, 0);
                        flag = 1;
                        }
                        else if((decision == 'R') || (decision == 'r')){
                            // Redo the last undone move
                            boardCopy(tmpBoard, board, dim);
                            addSkipper(turn, players, lastSkipper);
                            if(turn == 0){
                                turn++;
                            }
                            else{
                                turn = 0;
                            }
                        }
                    }
                }
                else{
                    printf("\n wrong\n");
                }
            }
            counter = 0;
            turn = 1;
        }
        // Computer's turn
        else{
            // Let the computer make a move
            board = computerMove(board, dim, players);
            turn = 0;
        }
    }
    
    // Free dynamically allocated memory
    for(i = 0; i < dim; i++) {
        free(tmpBoard[i]);
        free(previousBoard[i]);
    }
    free(tmpBoard);
    free(previousBoard);

    // If the game ends without a manual exit, display the winner
    if(!flag){
        system("cls");
        displayScore(players);
        printf("\n");
        getWinner(players);
    }
}



int main() {
    int **board, dim = 20;
	char decision;
	int priorityPlayer, players[2][MAX] = { {0} }, gameType;
	board = readFromFileGameBoard(board, &dim);
	int x;
	// Eðer oyun tahtasý boyutu 0 deðilse devam et
	if(dim != 0){
	    printf("1 - Manuel Mod\n2 - Automatic Mod\n3 - Last Game\n");
	    scanf("%d", &x);
	    switch(x){
	        case(1):
	            board = resetBoard(board, dim);
	            printf("Enter dimension of board: ");
	            scanf("%d", &dim);
	            createBoard(board, dim);
	            playerVsPlayer(board, dim, 0, players);
	            break;
	            
	        case(2):
	            board = resetBoard(board, dim);
	            printf("Enter dimension of board: ");
	            scanf("%d", &dim);
	            createBoard(board, dim);
	            playerVsComputer(board, dim, 0, players);
	            break;
	        
	        case(3):
	            // Oyun bilgilerini dosyadan oku
	            readFromFileGameInfo(&priorityPlayer, players, &gameType);
	            // Oyun türüne göre iþlem yap
	            if (gameType == 0) {
	                playerVsPlayer(board, dim, priorityPlayer, players);
	            } 
	            else{
	                playerVsComputer(board, dim, priorityPlayer, players);
	            }  
	            break; 
	    }
	}
	else{
	    printf("1 - Manuel Mod\n2 - Automatic Mod\n");
	    scanf("%d", &x);
	    switch(x){
	        case(1):
	            // Oyun tahtasýný sýfýrla ve boyutunu al
	            board = resetBoard(board, dim);
	            printf("Enter dimension of board: ");
	            scanf("%d", &dim);
	            // Oyun tahtasýný oluþtur
	            createBoard(board, dim);
	            // Ýki oyuncu arasýnda oyunu baþlat
	            playerVsPlayer(board, dim, 0, players);
	            break;
	            
	        case(2):
	            board = resetBoard(board, dim);
	            printf("Enter dimension of board: ");
	            scanf("%d", &dim);
	            createBoard(board, dim);
	            playerVsComputer(board, dim, 0, players);
	            break;
	    }
	}

	// Dinamik olarak ayrýlan belleði serbest býrak
	int i; 
	for (i = 0; i < dim; i++) {
	    free(board[i]);
	}
	free(board);
	
	// Programýn baþarýyla sonlandýðýný belirt
	return 0;

}

