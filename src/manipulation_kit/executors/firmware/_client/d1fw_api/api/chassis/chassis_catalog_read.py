from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_catalog_read_response_200 import ChassisCatalogReadResponse200
from ...models.chassis_catalog_request_type_0 import ChassisCatalogRequestType0
from ...models.chassis_catalog_request_type_1 import ChassisCatalogRequestType1
from ...models.chassis_catalog_request_type_2 import ChassisCatalogRequestType2
from ...models.chassis_catalog_request_type_3 import ChassisCatalogRequestType3
from ...models.chassis_catalog_request_type_4 import ChassisCatalogRequestType4
from ...models.chassis_catalog_request_type_5 import ChassisCatalogRequestType5
from ...models.chassis_catalog_request_type_6 import ChassisCatalogRequestType6
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ChassisCatalogRequestType0
    | ChassisCatalogRequestType1
    | ChassisCatalogRequestType2
    | ChassisCatalogRequestType3
    | ChassisCatalogRequestType4
    | ChassisCatalogRequestType5
    | ChassisCatalogRequestType6,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/catalog/read",
    }

    if (
        isinstance(body, ChassisCatalogRequestType0)
        or isinstance(body, ChassisCatalogRequestType1)
        or isinstance(body, ChassisCatalogRequestType2)
        or isinstance(body, ChassisCatalogRequestType3)
        or isinstance(body, ChassisCatalogRequestType4)
        or isinstance(body, ChassisCatalogRequestType5)
    ):
        _kwargs["json"] = body.to_dict()
    else:
        _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisCatalogReadResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisCatalogReadResponse200.from_dict(response.json())

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
) -> Response[ChassisCatalogReadResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCatalogRequestType0
    | ChassisCatalogRequestType1
    | ChassisCatalogRequestType2
    | ChassisCatalogRequestType3
    | ChassisCatalogRequestType4
    | ChassisCatalogRequestType5
    | ChassisCatalogRequestType6,
) -> Response[ChassisCatalogReadResponse200 | ErrorEnvelope]:
    """Read task, plan, cyclic settings or USB catalog

     Read-only fixed selector. No task execution, storage mount or writes. Scene and plan identifiers are
    explicit. The response preserves vendor JSON without assuming a generation-specific schema. Null is
    an explicit vendor result, not a fabricated empty catalog. Vendor refusals retain their numeric
    codes. Allowed while soft-killed.

    Args:
        body (ChassisCatalogRequestType0 | ChassisCatalogRequestType1 | ChassisCatalogRequestType2
            | ChassisCatalogRequestType3 | ChassisCatalogRequestType4 | ChassisCatalogRequestType5 |
            ChassisCatalogRequestType6): Exactly seven supported reads. No caller-selected endpoint is
            accepted.

            This is where a fixed, parameterless or identifier-selected vendor
            DOCUMENT read belongs, rather than a route of its own: the daemon's own
            chassis routes publish the daemon's model of the base (its state, its
            navigation, its settings), while these publish a vendor document verbatim,
            with its provenance, for a consumer that needs the vendor's own wording.
            `RosStatus` and `ChargingStatToday` are read-only reads of exactly that
            shape, which is why they are variants here.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisCatalogReadResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCatalogRequestType0
    | ChassisCatalogRequestType1
    | ChassisCatalogRequestType2
    | ChassisCatalogRequestType3
    | ChassisCatalogRequestType4
    | ChassisCatalogRequestType5
    | ChassisCatalogRequestType6,
) -> ChassisCatalogReadResponse200 | ErrorEnvelope | None:
    """Read task, plan, cyclic settings or USB catalog

     Read-only fixed selector. No task execution, storage mount or writes. Scene and plan identifiers are
    explicit. The response preserves vendor JSON without assuming a generation-specific schema. Null is
    an explicit vendor result, not a fabricated empty catalog. Vendor refusals retain their numeric
    codes. Allowed while soft-killed.

    Args:
        body (ChassisCatalogRequestType0 | ChassisCatalogRequestType1 | ChassisCatalogRequestType2
            | ChassisCatalogRequestType3 | ChassisCatalogRequestType4 | ChassisCatalogRequestType5 |
            ChassisCatalogRequestType6): Exactly seven supported reads. No caller-selected endpoint is
            accepted.

            This is where a fixed, parameterless or identifier-selected vendor
            DOCUMENT read belongs, rather than a route of its own: the daemon's own
            chassis routes publish the daemon's model of the base (its state, its
            navigation, its settings), while these publish a vendor document verbatim,
            with its provenance, for a consumer that needs the vendor's own wording.
            `RosStatus` and `ChargingStatToday` are read-only reads of exactly that
            shape, which is why they are variants here.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisCatalogReadResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCatalogRequestType0
    | ChassisCatalogRequestType1
    | ChassisCatalogRequestType2
    | ChassisCatalogRequestType3
    | ChassisCatalogRequestType4
    | ChassisCatalogRequestType5
    | ChassisCatalogRequestType6,
) -> Response[ChassisCatalogReadResponse200 | ErrorEnvelope]:
    """Read task, plan, cyclic settings or USB catalog

     Read-only fixed selector. No task execution, storage mount or writes. Scene and plan identifiers are
    explicit. The response preserves vendor JSON without assuming a generation-specific schema. Null is
    an explicit vendor result, not a fabricated empty catalog. Vendor refusals retain their numeric
    codes. Allowed while soft-killed.

    Args:
        body (ChassisCatalogRequestType0 | ChassisCatalogRequestType1 | ChassisCatalogRequestType2
            | ChassisCatalogRequestType3 | ChassisCatalogRequestType4 | ChassisCatalogRequestType5 |
            ChassisCatalogRequestType6): Exactly seven supported reads. No caller-selected endpoint is
            accepted.

            This is where a fixed, parameterless or identifier-selected vendor
            DOCUMENT read belongs, rather than a route of its own: the daemon's own
            chassis routes publish the daemon's model of the base (its state, its
            navigation, its settings), while these publish a vendor document verbatim,
            with its provenance, for a consumer that needs the vendor's own wording.
            `RosStatus` and `ChargingStatToday` are read-only reads of exactly that
            shape, which is why they are variants here.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisCatalogReadResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisCatalogRequestType0
    | ChassisCatalogRequestType1
    | ChassisCatalogRequestType2
    | ChassisCatalogRequestType3
    | ChassisCatalogRequestType4
    | ChassisCatalogRequestType5
    | ChassisCatalogRequestType6,
) -> ChassisCatalogReadResponse200 | ErrorEnvelope | None:
    """Read task, plan, cyclic settings or USB catalog

     Read-only fixed selector. No task execution, storage mount or writes. Scene and plan identifiers are
    explicit. The response preserves vendor JSON without assuming a generation-specific schema. Null is
    an explicit vendor result, not a fabricated empty catalog. Vendor refusals retain their numeric
    codes. Allowed while soft-killed.

    Args:
        body (ChassisCatalogRequestType0 | ChassisCatalogRequestType1 | ChassisCatalogRequestType2
            | ChassisCatalogRequestType3 | ChassisCatalogRequestType4 | ChassisCatalogRequestType5 |
            ChassisCatalogRequestType6): Exactly seven supported reads. No caller-selected endpoint is
            accepted.

            This is where a fixed, parameterless or identifier-selected vendor
            DOCUMENT read belongs, rather than a route of its own: the daemon's own
            chassis routes publish the daemon's model of the base (its state, its
            navigation, its settings), while these publish a vendor document verbatim,
            with its provenance, for a consumer that needs the vendor's own wording.
            `RosStatus` and `ChargingStatToday` are read-only reads of exactly that
            shape, which is why they are variants here.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisCatalogReadResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
