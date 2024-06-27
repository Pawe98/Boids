# Ensure necessary packages are installed and loaded
required_packages <- c("dplyr", "data.table", "plotly", "crosstalk", "furrr", "tidyr", "factoextra")

install_if_missing <- function(pkg) {
  if (!requireNamespace(pkg, quietly = TRUE)) {
    install.packages(pkg)
  }
  library(pkg, character.only = TRUE)
}

lapply(required_packages, install_if_missing)

# Set the path to your directory with the CSV files
directory <- "C:\\Users\\pauld\\Desktop\\processing-4.3"

# Path to save the processed data
processed_data_path <- "C:\\Users\\pauld\\Desktop\\final_aggregated_data.rds"

# Function to count visible neighbors
count_visible_neighbors <- function(x) {
  if (x == "[]") {
    return(0)
  } else {
    x <- gsub("\\[|\\]", "", x)  # Remove square brackets
    if (nchar(x) == 0) {
      return(0)
    } else {
      return(length(unlist(strsplit(x, ";"))))
    }
  }
}

# Function to process CSV files and return aggregated data
process_csv_files <- function(directory) {
  csv_files <- list.files(path = directory, pattern = "\\.csv$", full.names = TRUE)
  cat("Anzahl der gefundenen CSV-Dateien: ", length(csv_files), "\n")
  
  # Parallel processing of files
  data_list <- furrr::future_map(csv_files, function(file) {
    cat("Processing file: ", file, "\n")
    
    # Read the CSV file
    data <- read.csv(file, stringsAsFactors = FALSE)
    
    # Ensure column names are correctly referenced
    expected_columns <- c("PermutationCounter", "desiredSeparation", "neighborDist", 
                          "separationWeight", "alignmentWeight", "cohesionWeight", 
                          "Visible.Neighbors")
    
    # Check if all expected columns are present
    if (!all(expected_columns %in% colnames(data))) {
      cat("Not all expected columns are present in the data.\n")
      return(NULL)  # Skip to the next iteration
    }
    
    # Select necessary columns
    data <- data[, expected_columns]
    
    # Calculate visible neighbors count
    data$VisibleNeighborsCount <- sapply(data$"Visible.Neighbors", count_visible_neighbors)
    
    # Calculate average VisibleNeighborsCount for this file
    avg_visible_neighbors <- mean(data$VisibleNeighborsCount, na.rm = TRUE)
    
    # Extract configuration parameters from the first row
    config <- data[1, c("PermutationCounter", "desiredSeparation", "neighborDist", 
                        "separationWeight", "alignmentWeight", "cohesionWeight")]
    
    # Add avg_visible_neighbors to config
    config$avg_visible_neighbors <- avg_visible_neighbors
    
    # Return config as a data frame
    return(config)
  })
  
  # Remove NULL entries (in case any files were skipped)
  data_list <- data_list[!sapply(data_list, is.null)]
  
  # Combine all data frames into one large data frame
  aggregated_data <- dplyr::bind_rows(data_list)
  
  # Aggregate the data by configuration parameters
  final_aggregated_data <- dplyr::group_by(aggregated_data, 
                                           PermutationCounter, desiredSeparation, neighborDist, 
                                           separationWeight, alignmentWeight, cohesionWeight) %>%
    dplyr::summarise(AverageVisibleNeighbors = mean(avg_visible_neighbors, na.rm = TRUE)) %>%
    dplyr::ungroup()
  
  return(final_aggregated_data)
}

# Load or process data
if (file.exists(processed_data_path)) {
  final_aggregated_data <- readRDS(processed_data_path)
} else {
  final_aggregated_data <- process_csv_files(directory)
  saveRDS(final_aggregated_data, processed_data_path)
}

# Perform PCA on numeric columns excluding 'AverageVisibleNeighbors'
numeric_cols <- c("cohesionWeight", "alignmentWeight", "separationWeight", "desiredSeparation", "neighborDist")
final_aggregated_data_numeric <- final_aggregated_data[, numeric_cols]

# Standardize the data
final_aggregated_data_numeric_std <- scale(final_aggregated_data_numeric)

# Perform PCA
pca_result <- stats::prcomp(final_aggregated_data_numeric_std, center = TRUE, scale. = TRUE)

# Extract PCA components
pca_components <- as.data.frame(pca_result$x[, 1:2])  # Extract first two components

# Extract PCA loadings from the prcomp object
loadings <- pca_result$rotation[, 1:2]

# Get column names of the original variables
variable_names <- colnames(final_aggregated_data_numeric)

# Initialize empty vectors to store terms for PC1 and PC2
pc1_terms <- character(length(variable_names))
pc2_terms <- character(length(variable_names))

# Loop through each variable and populate terms for PC1 and PC2
for (i in seq_along(variable_names)) {
  if (loadings[i, 1] != 0) {
    pc1_terms[i] <- paste0(round(loadings[i, 1], 2), " * ", variable_names[i])
  }
  if (loadings[i, 2] != 0) {
    pc2_terms[i] <- paste0(round(loadings[i, 2], 2), " * ", variable_names[i])
  }
}

# Create the final formulas for PC1 and PC2
pc1_formula <- paste("PC1 =", paste(pc1_terms[pc1_terms != ""], collapse = " + "))
pc2_formula <- paste("PC2 =", paste(pc2_terms[pc2_terms != ""], collapse = " + "))


# Combine PCA components with original data
final_aggregated_data_pca <- cbind(final_aggregated_data, pca_components)

# Create hover text with customized labels
hover_text <- with(final_aggregated_data_pca, paste(
  "<b>Cohesion Weight:</b> ", format(cohesionWeight, digits = 2),
  "<br><b>Alignment Weight:</b> ", format(alignmentWeight, digits = 2),
  "<br><b>Separation Weight:</b> ", format(separationWeight, digits = 2),
  "<br><b>Neighbor Dist:</b> ", format(neighborDist, digits = 2),
  "<br><b>Desired Separation:</b> ", format(desiredSeparation, digits = 2),
  "<br><b>Average Visible Neighbors:</b> ", format(AverageVisibleNeighbors, digits = 2),
  sep = ""
))

# Create plotly 3D scatter plot with PCA components
plot <- plotly::plot_ly(
  final_aggregated_data_pca,
  x = ~PC1,
  y = ~PC2,
  z = ~AverageVisibleNeighbors,
  color = ~neighborDist,  # Use neighborDist for color
  size = ~desiredSeparation,  # Use desiredSeparation for marker size
  text = hover_text,
  type = "scatter3d",
  mode = "markers",
  marker = list(
    sizemode = "diameter",  # Set size mode to "diameter" to scale by radius
    sizeref = 8,          # Scale factor for marker sizes
    opacity = 0.8           # Adjust marker opacity if needed
  )
) %>%
  plotly::layout(
    scene = list(
      xaxis = list(title = pc1_formula),
      yaxis = list(title = pc2_formula),
      zaxis = list(title = "AverageVisibleNeighbors")
    ),
    width = 1200,  # Adjusted width
    height = 800   # Adjusted height
  )

plot
