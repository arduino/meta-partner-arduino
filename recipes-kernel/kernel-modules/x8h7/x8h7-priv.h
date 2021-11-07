/**
 * Portenta X8
 * X8 H7 communication protocol
 */


static struct spi_driver mcp251x_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = mcp251x_of_match,
		.pm = &mcp251x_can_pm_ops,
	},
	.id_table = mcp251x_id_table,
	.probe = mcp251x_can_probe,
	.remove = mcp251x_can_remove,
};
module_spi_driver(mcp251x_can_driver);

MODULE_AUTHOR("Chris Elston <celston@katalix.com>, "
	      "Christian Pellegrin <chripell@evolware.org>");
MODULE_DESCRIPTION("Microchip 251x/25625 CAN driver");
MODULE_LICENSE("GPL v2");
