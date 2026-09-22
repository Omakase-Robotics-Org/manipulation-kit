from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.charging_dock_put import ChargingDockPut
from ...models.chassis_maps_charging_dock_put_response_200 import (
    ChassisMapsChargingDockPutResponse200,
)
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    scene: str,
    *,
    body: ChargingDockPut,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "put",
        "url": "/v1/chassis/maps/{scene}/charging_dock".format(
            scene=quote(str(scene), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMapsChargingDockPutResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMapsChargingDockPutResponse200.from_dict(response.json())

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
) -> Response[ChassisMapsChargingDockPutResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: ChargingDockPut,
) -> Response[ChassisMapsChargingDockPutResponse200 | ErrorEnvelope]:
    """Set which of this map's waypoints is the charging dock

     Writes the base's charging-dock setting (the vendor's `setChargeInfo`): which of this map's
    waypoints the base returns to when it charges. `POST /v1/chassis/charge` is refused with `kind`
    `dock_unconfigured` until this has been written at least once. The setting written here is read back
    as `dock_point` in `GET /v1/chassis/charge`. Idempotent, and gated by the chassis soft-kill latch.

    Args:
        scene (str):
        body (ChargingDockPut): `PUT /v1/chassis/maps/{scene}/charging_dock`.

            The scene is the path segment, so only the waypoint is in the body: it
            names which of that map's waypoints the mobile base treats as its
            charging dock.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsChargingDockPutResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: ChargingDockPut,
) -> ChassisMapsChargingDockPutResponse200 | ErrorEnvelope | None:
    """Set which of this map's waypoints is the charging dock

     Writes the base's charging-dock setting (the vendor's `setChargeInfo`): which of this map's
    waypoints the base returns to when it charges. `POST /v1/chassis/charge` is refused with `kind`
    `dock_unconfigured` until this has been written at least once. The setting written here is read back
    as `dock_point` in `GET /v1/chassis/charge`. Idempotent, and gated by the chassis soft-kill latch.

    Args:
        scene (str):
        body (ChargingDockPut): `PUT /v1/chassis/maps/{scene}/charging_dock`.

            The scene is the path segment, so only the waypoint is in the body: it
            names which of that map's waypoints the mobile base treats as its
            charging dock.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsChargingDockPutResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        scene=scene,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: ChargingDockPut,
) -> Response[ChassisMapsChargingDockPutResponse200 | ErrorEnvelope]:
    """Set which of this map's waypoints is the charging dock

     Writes the base's charging-dock setting (the vendor's `setChargeInfo`): which of this map's
    waypoints the base returns to when it charges. `POST /v1/chassis/charge` is refused with `kind`
    `dock_unconfigured` until this has been written at least once. The setting written here is read back
    as `dock_point` in `GET /v1/chassis/charge`. Idempotent, and gated by the chassis soft-kill latch.

    Args:
        scene (str):
        body (ChargingDockPut): `PUT /v1/chassis/maps/{scene}/charging_dock`.

            The scene is the path segment, so only the waypoint is in the body: it
            names which of that map's waypoints the mobile base treats as its
            charging dock.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsChargingDockPutResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: ChargingDockPut,
) -> ChassisMapsChargingDockPutResponse200 | ErrorEnvelope | None:
    """Set which of this map's waypoints is the charging dock

     Writes the base's charging-dock setting (the vendor's `setChargeInfo`): which of this map's
    waypoints the base returns to when it charges. `POST /v1/chassis/charge` is refused with `kind`
    `dock_unconfigured` until this has been written at least once. The setting written here is read back
    as `dock_point` in `GET /v1/chassis/charge`. Idempotent, and gated by the chassis soft-kill latch.

    Args:
        scene (str):
        body (ChargingDockPut): `PUT /v1/chassis/maps/{scene}/charging_dock`.

            The scene is the path segment, so only the waypoint is in the body: it
            names which of that map's waypoints the mobile base treats as its
            charging dock.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsChargingDockPutResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            scene=scene,
            client=client,
            body=body,
        )
    ).parsed
