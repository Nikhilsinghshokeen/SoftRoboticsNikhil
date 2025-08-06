#!/bin/bash

# MediaPipeWPI Setup Script
# This script helps initialize and update the project with its submodules

echo "🚀 Setting up MediaPipeWPI project..."

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "❌ Git is not installed. Please install git first."
    exit 1
fi

# Initialize submodules if not already done
echo "📚 Initializing submodules..."
git submodule init
git submodule update

# Check if submodules were initialized successfully
if [ -d "wiki" ] && [ -f "wiki/Home.md" ]; then
    echo "✅ Wiki submodule initialized successfully!"
else
    echo "❌ Failed to initialize wiki submodule."
    exit 1
fi

echo ""
echo "🎉 Setup complete!"
echo ""
echo "📖 Next steps:"
echo "1. Read the README.md for quick start instructions"
echo "2. Check the wiki/ directory for detailed documentation:"
echo "   - wiki/Home.md - Main wiki index"
echo "   - wiki/Hardware-Setup.md - Hardware assembly guide"
echo "   - wiki/Software-Installation.md - Software setup guide"
echo "   - wiki/Troubleshooting.md - Common issues and solutions"
echo "   - wiki/Technical-Specifications.md - Technical details"
echo ""
echo "🔧 To update the wiki later, run:"
echo "   git submodule update --remote"
echo "" 