require_relative 'model'
Record.where(version: 10503).delete_all
