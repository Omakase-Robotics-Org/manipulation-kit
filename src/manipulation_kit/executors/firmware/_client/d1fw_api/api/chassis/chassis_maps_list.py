from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_maps_list_response_200 import ChassisMapsListResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/maps",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMapsListResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMapsListResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisMapsListResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisMapsListResponse200 | ErrorEnvelope]:
    """List the mobile base's saved maps

     The map collection: one entry per scene the base has stored, with the scene's origin, its metres-
    per-pixel resolution and how many waypoints its road network holds. Composed by the daemon from
    three vendor reads, so unlike this resource's `road` and `keep_outs` reads, its shape IS pinned by
    this document. This `chassis.maps.*` resource is the ONLY map surface of the REST, WebSocket and FFI
    frontends this document describes. The pinned `/d1` ROS 2 contract's own two map reads
    (`get_map_catalog`, `get_map_image`) are untouched by that and are served over ROS 2 and iceoryx2
    instead, not documented here: that contract belongs to the `omakase_msg` package of `d1-ros2`, not
    to this daemon, and is consumed by `test--agentic-omakase`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsListResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisMapsListResponse200 | ErrorEnvelope | None:
    """List the mobile base's saved maps

     The map collection: one entry per scene the base has stored, with the scene's origin, its metres-
    per-pixel resolution and how many waypoints its road network holds. Composed by the daemon from
    three vendor reads, so unlike this resource's `road` and `keep_outs` reads, its shape IS pinned by
    this document. This `chassis.maps.*` resource is the ONLY map surface of the REST, WebSocket and FFI
    frontends this document describes. The pinned `/d1` ROS 2 contract's own two map reads
    (`get_map_catalog`, `get_map_image`) are untouched by that and are served over ROS 2 and iceoryx2
    instead, not documented here: that contract belongs to the `omakase_msg` package of `d1-ros2`, not
    to this daemon, and is consumed by `test--agentic-omakase`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsListResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisMapsListResponse200 | ErrorEnvelope]:
    """List the mobile base's saved maps

     The map collection: one entry per scene the base has stored, with the scene's origin, its metres-
    per-pixel resolution and how many waypoints its road network holds. Composed by the daemon from
    three vendor reads, so unlike this resource's `road` and `keep_outs` reads, its shape IS pinned by
    this document. This `chassis.maps.*` resource is the ONLY map surface of the REST, WebSocket and FFI
    frontends this document describes. The pinned `/d1` ROS 2 contract's own two map reads
    (`get_map_catalog`, `get_map_image`) are untouched by that and are served over ROS 2 and iceoryx2
    instead, not documented here: that contract belongs to the `omakase_msg` package of `d1-ros2`, not
    to this daemon, and is consumed by `test--agentic-omakase`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsListResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisMapsListResponse200 | ErrorEnvelope | None:
    """List the mobile base's saved maps

     The map collection: one entry per scene the base has stored, with the scene's origin, its metres-
    per-pixel resolution and how many waypoints its road network holds. Composed by the daemon from
    three vendor reads, so unlike this resource's `road` and `keep_outs` reads, its shape IS pinned by
    this document. This `chassis.maps.*` resource is the ONLY map surface of the REST, WebSocket and FFI
    frontends this document describes. The pinned `/d1` ROS 2 contract's own two map reads
    (`get_map_catalog`, `get_map_image`) are untouched by that and are served over ROS 2 and iceoryx2
    instead, not documented here: that contract belongs to the `omakase_msg` package of `d1-ros2`, not
    to this daemon, and is consumed by `test--agentic-omakase`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsListResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
